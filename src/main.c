#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <inttypes.h>
#include <string.h>
#define _GNU_SOURCE
#include <getopt.h>
#include <glib.h>

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>

#include <lcmtypes/bot2_core.h>
#include <lcmtypes/hr_lcmtypes.h>


#if DEBUG
#define dbg(...) do { fprintf(stderr, "[%s:%d] ", __FILE__, __LINE__); \
                      fprintf(stderr, __VA_ARGS__); } while(0)
#else
#define dbg(...) 
#endif

// Both the ROBOT_STATE_NAMES and WARNINGS Must match the constants defined in erlcm_robot_state_command_t.lcm
static char *robot_state_name[6]={"UNDEFINED","RUN","STANDBY","STOP","MANUAL","ERROR"};

static char *warnings[]={"FAULT NAV","FAULT ACTUATION","FAULT SENSORS","FAULT TIMEOUT", // 16 "device faults"
                         "FAULT ESTOP","","","","","","","","","","",""
                         "FAULT HEARTBEAT","FAULT CONTROL","FAULT PERCEPT", // 16 "low-level software faults"
                         "","","","","","","","","","","","","",
                         "FAULT SHOUT","FAULT HUMAN","FAULT OVERRIDE", // 16 "high-level faults"
                         "","","","","","","","","","","","","",
                         "","","","","","","",""};

#define PUBLISH_STATE_HZ 20
#define WATCHDOG_USEC 500000
//#define STANDBY_GRACE_USEC 5000000 // Time spent in STANDBY before RUN
#define STANDBY_GRACE_USEC 0 // Time spent in STANDBY before RUN

typedef struct _watchdog_t {
    int64_t last_utime;
    char *name;
    char *channel;
    char *comment;
    int64_t tol_usec;
    int32_t state;
} watchdog_t;


typedef struct _state_t {
    lcm_t *lcm;
    BotParam *param;
    char *name;
    int verbose;
    int developer_mode;
    GHashTable *watchdog_hash;
    erlcm_robot_status_t status; 
    erlcm_robot_status_t prev;
    int64_t standby_utime;
} state_t;


static void
robot_status_publish_state(state_t *self) 
{
    self->status.utime = bot_timestamp_now();
    erlcm_robot_status_t_publish (self->lcm, "ROBOT_STATUS", &self->status);
}


static int
robot_status_change_state(state_t *self, const int64_t utime, const int8_t state,  
                          const int64_t faults, const char *comment)
{
    if (self->verbose)
        printf("robot_change_state():%d faults:%"PRId64" comment:%s\n",
                state,self->status.faults,comment?comment:"<none>");

    // track transitions
    if (state!=self->status.state) {
        if (self->prev.comment)
            free(self->prev.comment);
        self->prev = self->status;
    }

    self->status.utime = utime;
    self->status.state = state;
    self->status.faults = faults;

    if (comment)
        self->status.comment=strdup(comment);
    else
        self->status.comment=NULL;

    return self->status.state;
}

static char *
spew_out_warnings(state_t *self, int64_t faults, int8_t cmd) {
    char * first_warning=NULL;
    erlcm_comment_t comment;
    comment.utime = bot_timestamp_now();
    for (int i=0; i < 64; i++) {
        if ( ( (int64_t)1 << i) & faults ) {
            if (!first_warning)
                first_warning=warnings[i];

            char buff[256];

            //snprintf(buff,256,"%s prevented by:[0x%08x%08x]%s",robot_state_name[cmd],(i>31?(1<<(i-32)):0),(i<32?(1<<i):0),warnings[i]);
            snprintf(buff,256,"%s prevented by:[0x%08x%08x]",robot_state_name[cmd],(i>31?(1<<(i-32)):0),(i<32?(1<<i):0));
            comment.comment = buff;
            printf("[%"PRId64"]: %s\n",comment.utime,comment.comment);
            erlcm_comment_t_publish(self->lcm,"COMMENT",&comment);
        }
    }
    return first_warning;
}

static int
robot_status_handle_event(state_t *self, const erlcm_robot_state_command_t *cmd)
{

    char *comment = cmd->comment;
    if (self->verbose) {
        printf("robot_status_handle_event():state: %d faults:%"PRId64" comment:%s\n",
               self->status.state, self->status.faults, self->status.comment?self->status.comment:"<none>");
        printf("robot_status_handle_event():cmd: %d faults:%"PRId64"(mask:%"PRId64") %s:%s\n",
               cmd->state, cmd->faults, cmd->fault_mask, cmd->sender?cmd->sender:"<none>", 
               cmd->comment?cmd->comment:"<none>");
    }

    // Add existing faults to new faults then process consquences below.
    int64_t faults = self->status.faults | cmd->faults;
    

    // TODO: add some sanity check clearing faults.
    faults &= ~cmd->fault_mask;


    // Command is overrride and we are not in manual (override):
    // Commanded to go to MANUAL (OVERRIDE) when in STOP
    if ((cmd->state==ERLCM_ROBOT_STATUS_T_STATE_MANUAL) &&
             (self->status.state==ERLCM_ROBOT_STATUS_T_STATE_STOP)) {

        // Change states if none of the active faults are consistent with those preventing MANUAL
        if ( !(faults & ERLCM_ROBOT_STATUS_T_FAULTS_PREVENTING_MANUAL)) {
            // so switch to override:
            return robot_status_change_state(self, cmd->utime, cmd->state, faults, comment);
        }
        else {
            // override rejected because of 
            char *warning =
                spew_out_warnings (self, faults & ERLCM_ROBOT_STATUS_T_FAULTS_PREVENTING_MANUAL, cmd->state);
            if (warning) 
                comment = warning;
        }
            
    }

    // Upon receiving a RUN command, we transition to by
    //  1) STATE=STOP,    CMD=RUN       ----> STATE=STANDBY
    //  2) STATE=STANDBY, CMD=UNDEFINED ----> STATE=STANDBY while dt < countdown (CMD published by self timer)
    //                                  ----> STATE=RUN     if    dt > countdown

    // Faults preventing run transition:
    // All faults except these:
    if ( !(faults & ERLCM_ROBOT_STATUS_T_FAULTS_PREVENTING_STANDBY)) { 
        // 2. The UNDEFINED command comes from the self timer event. If we are in STANDBY => go to run yet?
        if ( (cmd->state == ERLCM_ROBOT_STATUS_T_STATE_UNDEFINED) &&
            (self->status.state == ERLCM_ROBOT_STATUS_T_STATE_STANDBY)) {
            char mycomment[256];
            int64_t dt = cmd->utime - self->prev.utime;
            if (dt<STANDBY_GRACE_USEC) {
                int64_t timer = (STANDBY_GRACE_USEC-dt)/1e6;
                snprintf(mycomment,256,"In...%"PRId64" sec.",timer);
                uint8_t *bytes=(uint8_t*)&faults;
                bytes[7]=(bytes[7]&0x0f)+(0xf0&(timer<<4));
                //printf("timer:%"PRId64" faults:%"PRId64" faults2:%"PRId64"\n",timer,faults,(timer<<60));
                return robot_status_change_state (self, cmd->utime, ERLCM_ROBOT_STATUS_T_STATE_STANDBY, faults, mycomment);
            }
            else if (dt<(STANDBY_GRACE_USEC+1e6)) {
                snprintf(mycomment,256,"Active!");
                return robot_status_change_state(self, cmd->utime, ERLCM_ROBOT_STATUS_T_STATE_RUN, faults, mycomment);
            }
            else {
                printf("ERROR: standby wait time some whacky number: %"PRId64" \n",dt);
            }
        }
        if ((cmd->state == ERLCM_ROBOT_STATUS_T_STATE_RUN) &&
            (self->status.state == ERLCM_ROBOT_STATUS_T_STATE_RUN)) {
            // we are in RUN and the command is RUN so it's a nop.
            return self->status.state;
        }
    }

    

    // We have no fault preventing standby (it's okay if a human is near. 
    // ie. the guy leaving the forklift, we just won't go into run.)
    // All faults except these:
    if ( !(faults & ERLCM_ROBOT_STATUS_T_FAULTS_PREVENTING_STANDBY)) { 
        // only valid state changes are:
        // 1. RUN command and stopped => go to standby.
        if (cmd->state==ERLCM_ROBOT_STATUS_T_STATE_RUN) {
            if ((self->status.state==ERLCM_ROBOT_STATUS_T_STATE_STOP)||
                (self->status.state==ERLCM_ROBOT_STATUS_T_STATE_STANDBY)) {                
                return robot_status_change_state (self, cmd->utime, ERLCM_ROBOT_STATUS_T_STATE_STANDBY, faults, cmd->comment);
            }
        }
    }
    else {
        // run rejected because of :
        if ((cmd->state==ERLCM_ROBOT_STATUS_T_STATE_RUN)&&((self->status.state==ERLCM_ROBOT_STATUS_T_STATE_STOP)||
                                                          (self->status.state==ERLCM_ROBOT_STATUS_T_STATE_STANDBY))) {                
            char *warning =
                spew_out_warnings( self, faults & ERLCM_ROBOT_STATUS_T_FAULTS_PREVENTING_STANDBY, cmd->state);
            if (warning) 
                comment = warning;
        }
    }
    

    // Check if the faults are ok in the current run mode:
    if (cmd->state != ERLCM_ROBOT_STATUS_T_STATE_STOP) {
        // OVERRIDE:
        if ((self->status.state == ERLCM_ROBOT_STATUS_T_STATE_MANUAL)&&
            ( !(faults & ERLCM_ROBOT_STATUS_T_FAULTS_OK_IN_MANUAL))) { 
            return self->status.state;
        }        
        // RUN:
        if ((self->status.state == ERLCM_ROBOT_STATUS_T_STATE_RUN) &&
            (!(faults & ERLCM_ROBOT_STATUS_T_FAULTS_OK_IN_MANUAL))) { 
            return self->status.state;
        }        
        // STANDBY:
        if ((self->status.state == ERLCM_ROBOT_STATUS_T_STATE_STANDBY)&&
            (!(faults & ERLCM_ROBOT_STATUS_T_FAULTS_OK_IN_STANDBY))) { 
            return self->status.state;
        }        

/*
    if ((cmd->state==ERLCM_ROBOT_STATUS_T_STATE_ERROR)&&
        (cmd->faults&ARLCM_ROBOT_STATUS_T_FAULT_SHOUT))
        printf("found shout\n");
    if ((cmd->state==ERLCM_ROBOT_STATUS_T_STATE_ERROR)&&
        (faults&ARLCM_ROBOT_STATUS_T_FAULT_SHOUT))
        printf("not masked shout\n");
*/
        // developer mode so if only fault is seat bit just set the bit
        if (self->developer_mode&&(!faults)) {
            self->status.faults=faults;
            return self->status.state;
        }

    }

    // throw away timer events
    if (cmd->state==ERLCM_ROBOT_STATUS_T_STATE_UNDEFINED)
        return self->status.state;

    
  
    // If we are here all the above didn't apply.
    // Either we have faults or requested to stop.
    // Either way STOP.
    return robot_status_change_state (self, cmd->utime, ERLCM_ROBOT_STATUS_T_STATE_STOP, faults,cmd->comment);

}

static void on_robot_state_command(const lcm_recv_buf_t *rbuf, const char *channel, 
                                   const erlcm_robot_state_command_t *cmd, void *user)
{
    state_t *self = (state_t*) user;
    robot_status_handle_event (self, cmd); 
}

static void on_heartbeat(const lcm_recv_buf_t *rbuf, const char *channel, 
                         const erlcm_heartbeat_t *msg, void *user)
{
    state_t *self = (state_t*) user;
    watchdog_t *w = g_hash_table_lookup(self->watchdog_hash, channel);
    if (!w) {
        if (self->verbose) {
            printf("INFO: unexpected heartbeat from:%s\n",channel);
            printf("INFO: should this be in the robot_status configfile?\n");
        }
        w = (watchdog_t *) calloc(1,sizeof(watchdog_t));
        w->name=strdup(channel);
        w->channel=strdup(channel);
        if (msg->tol_utime) 
            w->tol_usec = msg->tol_utime;
        else
            w->tol_usec = 1e6;
            
        g_hash_table_insert (self->watchdog_hash, w->channel, w);
    }
    w->last_utime = msg->utime;
    w->state=msg->state;
    if (w->comment)
        free(w->comment);
    w->comment=strdup(msg->comment);

}

static void on_message(const lcm_recv_buf_t *rbuf, const char *channel, void *user)
{
    state_t *self = (state_t*) user;
    watchdog_t *w = g_hash_table_lookup (self->watchdog_hash, channel);
    if (!w && self->verbose) {
        printf("INFO: unexpected message from:%s\n",channel);
        printf("INFO: should this be in the robot_status configfile?\n");
        return;
    }
    w->last_utime = rbuf->recv_utime;
    w->state = ERLCM_HEARTBEAT_T_STATE_UNKNOWN;    
}



// Check basic health of robot
int 
robot_status_check_watchdogs(state_t *self)
{
    int64_t now = bot_timestamp_now();

    GList *watchdogs = g_hash_table_get_values(self->watchdog_hash);
    for (GList *iter = watchdogs; iter; iter=iter->next) {
        watchdog_t *w = (watchdog_t *) iter->data;
        int64_t dt = w->last_utime-now;
        
        if ((w->state == ERLCM_HEARTBEAT_T_STATE_ERROR) ||
            (w->state == ERLCM_HEARTBEAT_T_STATE_WARN) ) {
            // heartbeat status bad
            char comment[256];
            snprintf(comment,256,"HEART [%s] BAD STATUS:%d comment:%s",w->name,w->state, w->comment);
            erlcm_robot_state_command_t cmd;
            cmd.utime = now;
            cmd.state = ERLCM_ROBOT_STATUS_T_STATE_ERROR;
            cmd.faults = ERLCM_ROBOT_STATUS_T_FAULT_HEARTBEAT;
            cmd.fault_mask= ERLCM_ROBOT_STATUS_T_FAULT_MASK_NO_CHANGE;
            cmd.comment = comment;
            cmd.sender = w->name;
            robot_status_handle_event (self, &cmd);
            return 1;
        }
        if (abs(dt)>w->tol_usec) {
            // watch dog has tripped out
            char comment[256];
            snprintf(comment,256,"HEART [%s]: TIMEOUT:%f (tol:%f)",w->name,dt*1.0e-6,w->tol_usec*1.0e-6);
            // heartbeat status bad
            erlcm_robot_state_command_t cmd;
            cmd.utime = now;
            cmd.state = ERLCM_ROBOT_STATUS_T_STATE_ERROR;
            cmd.faults = ERLCM_ROBOT_STATUS_T_FAULT_HEARTBEAT;
            cmd.fault_mask= ERLCM_ROBOT_STATUS_T_FAULT_MASK_NO_CHANGE;
            cmd.sender = self->name;
            cmd.comment = comment;
            robot_status_handle_event (self, &cmd);
            return 1;
        }
    }

    return 0;
}

static gboolean
on_timer(gpointer user_data) 
{
    state_t *self = (state_t *) user_data;

    // send timer event
    erlcm_robot_state_command_t cmd;
    cmd.utime = bot_timestamp_now();
    cmd.state = ERLCM_ROBOT_STATUS_T_STATE_UNDEFINED;
    cmd.faults = ERLCM_ROBOT_STATUS_T_FAULT_NONE;
    cmd.fault_mask = ERLCM_ROBOT_STATUS_T_FAULT_MASK_NO_CHANGE;
    cmd.comment = "";
    cmd.sender = self->name;
    robot_status_handle_event(self, &cmd);

    // Check the watchdog timers
    robot_status_check_watchdogs (self);

    robot_status_publish_state(self);
    return TRUE;
}


static void
robot_status_destroy(state_t *self)
{
    if (self) 
        free(self);
}

state_t *
robot_status_create()
{
    state_t *self = (state_t*)calloc(1, sizeof(state_t));
    if (!self) {
        dbg("Error: robot_status_create() failed to allocate self\n");
        goto fail;
    }
    self->name = strdup("ROBOT_STATUS");
    self->status.state=ERLCM_ROBOT_STATUS_T_STATE_STOP;
    self->status.comment=strdup("");
    self->prev.comment=strdup("");


    /* LCM */
    self->lcm = bot_lcm_get_global (NULL);
    if (!self->lcm) {
        dbg("Error: robot_status_create() failed to get global lcm object\n");
        goto fail;
    }
    
    bot_glib_mainloop_attach_lcm (self->lcm);

    self->watchdog_hash = g_hash_table_new(g_str_hash,g_str_equal);

    // Subscribe to LCM messages
    erlcm_robot_state_command_t_subscribe(self->lcm, "ROBOT_STATE_COMMAND", on_robot_state_command, self);
    erlcm_heartbeat_t_subscribe(self->lcm, "HEARTBEAT.*", on_heartbeat, self);

    // for each process in list:
    /* config */
    self->param = bot_param_new_from_server (self->lcm, 0);
    if (!self->param) {
        dbg("Error: robot_status_create() fail to get global param object\n");
        goto fail;
    }
    char **watch_channels = 
        bot_param_get_str_array_alloc (self->param, "robot_status.watch_channels");
    for (char **chan=watch_channels; (chan && *chan); chan++) { 
        watchdog_t *w = (watchdog_t *) calloc(1,sizeof(watchdog_t));
        w->name=strdup(*chan);
        w->channel=strdup(*chan);
        w->tol_usec = 5e5;
        g_hash_table_insert(self->watchdog_hash,w->channel,w);
        lcm_subscribe(self->lcm, w->channel, on_message, self);
    }
    if (watch_channels)
        bot_param_str_array_free(watch_channels);

    g_timeout_add(1000.0/PUBLISH_STATE_HZ, on_timer, self);

    return self; 
fail:
    robot_status_destroy(self);
    return NULL;
}


static void usage()
{
    fprintf (stderr, "usage: robot [options]\n"
             "\n"
             "  -h, --help             shows this help text and exits\n"
             "  -v, --verbose          be verbose\n"
             "  -D, --developer-mode   developer mode (expects developer in seat)\n"
        );
}


int main(int argc, char *argv[])
{
    setlinebuf (stdout);
    
    char *optstring = "hvD";
    char c;
    struct option long_opts[] = {
        {"help", no_argument, 0, 'h'},
        {"verbose", no_argument, 0, 'v'},
        {"developer-mode", no_argument, 0, 'D'},
        {0, 0, 0, 0}};

    state_t *self = robot_status_create();
    if (!self)
        return 1;
    
    while ((c = getopt_long (argc, argv, optstring, long_opts, 0)) >= 0)
    {
        switch (c) 
        {
        case 'D':
            self->developer_mode = 1;
            break;
        case 'v':
            self->verbose = 1;
            break;
        case 'h':
        default:
            usage();
            return 1;
        }
    }


    /* Main Loop */
    GMainLoop *main_loop = g_main_loop_new(NULL, FALSE);
    if (!main_loop) {
        dbg("Error: Failed to create the main loop\n");
        return -1;
    }

    /* sit and wait for messages */
    g_main_loop_run(main_loop);
    
    /* clean */
    robot_status_destroy(self);
    return 0;
}
