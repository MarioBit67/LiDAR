#include "TAndroid.h"
#include "libDiscipline.h"

/* NativeActivity entry. The glue runs android_main on its own thread with a looper that multiplexes
 * the activity commands, input and the sensor queue; the port turns each into a raw sink event. */

enum {
   mainPollMs = 50
};

//--------------------------------------------------------------------------------
static void mainOnCommand(struct android_app *app, LONG cmd)
{
   ((TAndroid *)app->userData)->HandleCommand(cmd);
}

//--------------------------------------------------------------------------------
static LONG mainOnInput(struct android_app *app, AInputEvent *e)
{
   return ((TAndroid *)app->userData)->HandleInput(e);
}

//--------------------------------------------------------------------------------
extern "C" void androidMain(struct android_app *app) // linked as android_main (--defsym)
{
   TAndroid port(app);

   app->userData = &port;
   app->onAppCmd = mainOnCommand;
   app->onInputEvent = mainOnInput;
   appStart(&port);
   while (!app->destroyRequested)
   {
      int                         events = 0;
      struct android_poll_source *source = NULL;
      int                         id = ALooper_pollOnce(mainPollMs, NULL, &events, (LPVOID *)&source);

      if ((id == LOOPER_ID_MAIN || id == LOOPER_ID_INPUT) && source)
         source->process(app, source);
      else if (id == LOOPER_ID_USER)
         port.PumpSensors();
      port.Tick();
   }
   appStop();
}

//--------------------------------------------------------------------------------
