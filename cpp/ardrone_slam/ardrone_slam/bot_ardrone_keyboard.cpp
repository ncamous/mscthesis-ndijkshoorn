#include "bot_ardrone_keyboard.h"
#include <string>
#include "bot_ardrone.h"
#include "global.h"

using namespace std;



bot_ardrone** bot_ardrone_keyboard::bots	= NULL;
int bot_ardrone_keyboard::nr_bots			= 0;
float bot_ardrone_keyboard::vel				= 0.0f;
bool bot_ardrone_keyboard::keypressed[255] = {false};
HHOOK bot_ardrone_keyboard::hHook;


LRESULT CALLBACK LowLevelKeyboardProc( int nCode, WPARAM wParam, LPARAM lParam )
{
	int i;
	bool ignore;

	KBDLLHOOKSTRUCT	*kd = (KBDLLHOOKSTRUCT *)lParam;

	ignore = (wParam == WM_KEYDOWN && bot_ardrone_keyboard::keypressed[kd->vkCode] == 1);

	if (ignore)
		return CallNextHookEx(bot_ardrone_keyboard::hHook, nCode, wParam, lParam);


	bot_ardrone_keyboard::keypressed[kd->vkCode] = (wParam == WM_KEYDOWN);


	switch (kd->vkCode)
		{
		case VK_T: // T
			if (wParam == WM_KEYDOWN)
			{
				printf("Start recording\n");
				for(i=0; i<bot_ardrone_keyboard::nr_bots; i++)
					bot_ardrone_keyboard::bots[i]->set_record();
				//printf("Recording button disabled!\n");
			}
			break;

		case VK_RETURN:
			if (wParam == WM_KEYDOWN)
			{
				//stop_behavior = !stop_behavior;
				for(i=0; i<bot_ardrone_keyboard::nr_bots; i++)
					bot_ardrone_keyboard::bots[i]->set_slam(!bot_ardrone_keyboard::bots[i]->slam_state);
			}
			break;

		case VK_M:
			if (wParam == WM_KEYDOWN)
			{
				for(i=0; i<bot_ardrone_keyboard::nr_bots; i++)
				{
					bot_ardrone_keyboard::bots[i]->get_slam()->off(SLAM_MODE_MAP);
					bot_ardrone_keyboard::bots[i]->get_slam()->off(SLAM_MODE_ACCEL);
					//bot_ardrone_keyboard::bots[i]->get_slam()->off(SLAM_MODE_VEL);
					bot_ardrone_keyboard::bots[i]->get_slam()->off(SLAM_MODE_VISUALMOTION);
					bot_ardrone_keyboard::bots[i]->get_slam()->on(SLAM_MODE_VISUALLOC);
				}
			}

		case VK_R: // R (recover)
			if (wParam == WM_KEYDOWN)
			{
				for(i=0; i<bot_ardrone_keyboard::nr_bots; i++)
					bot_ardrone_keyboard::bots[i]->recover(wParam == WM_KEYDOWN);
			}
			break;

		case VK_NL_PLUS: // NL keyboard
		case VK_OEM_PLUS:
			if (wParam == WM_KEYDOWN)
			{
				bot_ardrone_keyboard::vel = min(1.0f, bot_ardrone_keyboard::vel+0.03f);
				printf("Keyboard velocity set to: %f\n", bot_ardrone_keyboard::vel);
			}
			break;

		case VK_NL_MINUS: // NL keyboard
		case VK_OEM_MINUS:
			if (wParam == WM_KEYDOWN)
			{
				bot_ardrone_keyboard::vel = max(0.03f, bot_ardrone_keyboard::vel-0.03f);
				printf("Keyboard velocity set to: %f\n", bot_ardrone_keyboard::vel);
			}
			break;

		case VK_UP:
			bot_ardrone_keyboard::set(wParam, BOT_ARDRONE_LinearVelocity, true);
			break;

		case VK_DOWN:
			bot_ardrone_keyboard::set(wParam, BOT_ARDRONE_LinearVelocity, false);
			break;

		case VK_LEFT:
			bot_ardrone_keyboard::set(wParam, BOT_ARDRONE_LateralVelocity, false);
			break;

		case VK_RIGHT:
			bot_ardrone_keyboard::set(wParam, BOT_ARDRONE_LateralVelocity, true);
			break;

		case VK_W:
			bot_ardrone_keyboard::set(wParam, BOT_ARDRONE_AltitudeVelocity, true);
			break;

		case VK_S:
			bot_ardrone_keyboard::set(wParam, BOT_ARDRONE_AltitudeVelocity, false);
			break;

		case VK_A:
			bot_ardrone_keyboard::set(wParam, BOT_ARDRONE_RotationalVelocity, false);
			break;

		case VK_D:
			bot_ardrone_keyboard::set(wParam, BOT_ARDRONE_RotationalVelocity, true);
			break;

		case VK_SPACE:
			if (wParam == WM_KEYDOWN)
			{
 				for(i=0; i<bot_ardrone_keyboard::nr_bots; i++)
				{
					if (bot_ardrone_keyboard::bots[i]->control.state == BOT_STATE_LANDED)
						bot_ardrone_keyboard::bots[i]->take_off();
					else
						bot_ardrone_keyboard::bots[i]->land();
				}
			}
			break;

		case VK_ESCAPE:
			if (wParam == WM_KEYDOWN)
			{
				stop_behavior = !stop_behavior;
				printf("Autonomous behavior: %s\n", stop_behavior ? "Stopped" : "Running");
			}
			/*
			exit_application = true;
			UnhookWindowsHookEx(bot_ardrone_keyboard::hHook);
			PostQuitMessage(0);
			*/
			break;
	}

	return CallNextHookEx(bot_ardrone_keyboard::hHook, nCode, wParam, lParam);
}


void bot_ardrone_keyboard::set(int action, int type, bool increment)
{
	int i;
	float val;

	for(i=0; i<bot_ardrone_keyboard::nr_bots; i++)
	{
		/* keyup */
		if (action == WM_KEYUP)
			val = 0.0f;

		/* keydown */
		else if (action == WM_KEYDOWN)
			val = increment ? bot_ardrone_keyboard::vel : -bot_ardrone_keyboard::vel;

		if (bot_ardrone_keyboard::bots[i]->control_get(BOT_ARDRONE_Velocity, type) != val)
		{
			bot_ardrone_keyboard::bots[i]->control_set(BOT_ARDRONE_Velocity, type, val);
			bot_ardrone_keyboard::bots[i]->control_update();
		}
	}
}


bot_ardrone_keyboard::bot_ardrone_keyboard(bot_ardrone **b, int nr_bots)
{
	bot_ardrone_keyboard::bots		= b;
	bot_ardrone_keyboard::nr_bots	= nr_bots;
	bot_ardrone_keyboard::vel		= BOT_ARDRONE_KEYBOARD_VEL;

	bot_ardrone_keyboard::hHook = SetWindowsHookEx( WH_KEYBOARD_LL, LowLevelKeyboardProc, NULL, 0 );
	if (!bot_ardrone_keyboard::hHook)
        MessageBoxA(NULL, "Unable to SetWindowsHookEx!", "Error!", MB_OK);                                            

	MSG msg;
	while (GetMessage(&msg, NULL, 0, 0))
	{
		TranslateMessage(&msg);
		DispatchMessage(&msg);
	}
}


bot_ardrone_keyboard::~bot_ardrone_keyboard()
{
}