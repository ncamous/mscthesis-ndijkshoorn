#include "bot_ardrone_keyboard.h"
#include <Windows.h>
#include <string>
#include "bot_ardrone.h"
#include "global.h"

using namespace std;

HHOOK hHook;


LRESULT CALLBACK LowLevelKeyboardProc( int nCode, WPARAM wParam, LPARAM lParam )
{
	int i;

	KBDLLHOOKSTRUCT	*kd = (KBDLLHOOKSTRUCT *)lParam;

	// http://delphi.about.com/od/objectpascalide/l/blvkc.htm

	switch (kd->vkCode)
	{
		case VK_T: // T
			if (wParam == WM_KEYDOWN)
			{
				printf("Start recording\n");
				for(i=0; i<keyboard_nr_bots; i++)
					keyboard_bot[i]->set_record();
				//printf("Recording button disabled!\n");
			}
			break;

		case VK_RETURN:
			if (wParam == WM_KEYDOWN)
			{
				//stop_behavior = !stop_behavior;
				for(i=0; i<keyboard_nr_bots; i++)
					keyboard_bot[i]->set_slam(!keyboard_bot[i]->slam_state);
			}
			break;

		case VK_M:
			if (wParam == WM_KEYDOWN)
			{
				for(i=0; i<keyboard_nr_bots; i++)
				{
					//keyboard_bot[i]->get_slam()->off(SLAM_MODE_MAP);
					//keyboard_bot[i]->get_slam()->off(SLAM_MODE_ACCEL);
					//keyboard_bot[i]->get_slam()->off(SLAM_MODE_VEL);
					keyboard_bot[i]->get_slam()->on(SLAM_MODE_VISUALMOTION);
				}
			}

		case VK_R: // R (recover)
			if (wParam == WM_KEYDOWN)
			{
				for(i=0; i<keyboard_nr_bots; i++)
					keyboard_bot[i]->recover(wParam == WM_KEYDOWN);
			}
			break;

		case VK_NL_PLUS: // NL keyboard
		case VK_OEM_PLUS:
			if (wParam == WM_KEYDOWN)
			{
				keyboard_vel = min(1.0f, keyboard_vel+0.03f);
				printf("Keyboard velocity set to: %f\n", keyboard_vel);
			}
			break;

		case VK_NL_MINUS: // NL keyboard
		case VK_OEM_MINUS:
			if (wParam == WM_KEYDOWN)
			{
				keyboard_vel = max(0.03f, keyboard_vel-0.03f);
				printf("Keyboard velocity set to: %f\n", keyboard_vel);
			}
			break;

		case VK_UP:
			keyboard_set(wParam, BOT_ARDRONE_LinearVelocity, true);
			break;

		case VK_DOWN:
			keyboard_set(wParam, BOT_ARDRONE_LinearVelocity, false);
			break;

		case VK_LEFT:
			keyboard_set(wParam, BOT_ARDRONE_LateralVelocity, false);
			break;

		case VK_RIGHT:
			keyboard_set(wParam, BOT_ARDRONE_LateralVelocity, true);
			break;

		case VK_W:
			keyboard_set(wParam, BOT_ARDRONE_AltitudeVelocity, true);
			break;

		case VK_S:
			keyboard_set(wParam, BOT_ARDRONE_AltitudeVelocity, false);
			break;

		case VK_A:
			keyboard_set(wParam, BOT_ARDRONE_RotationalVelocity, false);
			break;

		case VK_D:
			keyboard_set(wParam, BOT_ARDRONE_RotationalVelocity, true);
			break;

		case VK_SPACE:
			if (wParam == WM_KEYDOWN)
			{
				for(i=0; i<keyboard_nr_bots; i++)
				{
					if (keyboard_bot[i]->control.state == BOT_STATE_LANDED)
						keyboard_bot[i]->take_off();
					else
						keyboard_bot[i]->land();
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
			UnhookWindowsHookEx(hHook);
			PostQuitMessage(0);
			*/
			break;
	}

	return CallNextHookEx(hHook, nCode, wParam, lParam);
}


void keyboard_set(int action, int type, bool increment)
{
	int i;
	float val;

	for(i=0; i<keyboard_nr_bots; i++)
	{
		/* keyup */
		if (action == WM_KEYUP)
			val = 0.0f;

		/* keydown */
		else if (action == WM_KEYDOWN)
			val = increment ? keyboard_vel : -keyboard_vel;

		if (keyboard_bot[i]->control_get(BOT_ARDRONE_Velocity, type) != val)
		{
			keyboard_bot[i]->control_set(BOT_ARDRONE_Velocity, type, val);
			keyboard_bot[i]->control_update();
		}
	}
}


bot_ardrone_keyboard::bot_ardrone_keyboard(bot_ardrone **b, int nr_bots)
{
	keyboard_bot = b;
	keyboard_nr_bots = nr_bots;
	keyboard_vel = BOT_ARDRONE_KEYBOARD_VEL;

	hHook = SetWindowsHookEx( WH_KEYBOARD_LL, LowLevelKeyboardProc, NULL, 0 );
	if (!hHook)
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