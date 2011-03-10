#include "bot_ardrone_keyboard.h"
#include <Windows.h>
#include <string>
#include "bot_ardrone.h"
#include "global.h"

using namespace std;

HHOOK hHook;


LRESULT CALLBACK LowLevelKeyboardProc( int nCode, WPARAM wParam, LPARAM lParam   )
{
	if (nCode == HC_ACTION)
	{
		if (wParam == WM_KEYDOWN)
		{
			KBDLLHOOKSTRUCT	*kd = (KBDLLHOOKSTRUCT *)lParam;

			switch (kd->vkCode)
			{
				case VK_UP:
					keyboard_bot->control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LinearVelocity, BOT_ARDRONE_KEYBOARD_VEL_MIN);
					break;

				case VK_DOWN:
					keyboard_bot->control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LinearVelocity, BOT_ARDRONE_KEYBOARD_VEL_MAX);
					break;

				case VK_LEFT:
					keyboard_bot->control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LateralVelocity, BOT_ARDRONE_KEYBOARD_VEL_MIN);
					break;

				case VK_RIGHT:
					keyboard_bot->control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LateralVelocity, BOT_ARDRONE_KEYBOARD_VEL_MAX);
					break;

				case VK_Q:
					keyboard_bot->control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_AltitudeVelocity, 1.0f);
					break;

				case VK_A:
					keyboard_bot->control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_AltitudeVelocity, -1.0f);
					break;

				case VK_LSHIFT:
					keyboard_bot->control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_RotationalVelocity, BOT_ARDRONE_KEYBOARD_VEL_MIN);
					break;

				case VK_RSHIFT:
					keyboard_bot->control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_RotationalVelocity, BOT_ARDRONE_KEYBOARD_VEL_MAX);
					break;

				case VK_SPACE:
					if (keyboard_bot->control.landed)
						keyboard_bot->take_off();
					else
						keyboard_bot->land();
					// new landed state if set in the bot_ardrone class
					break;
			}

		}
		else if (wParam == WM_KEYUP)
		{
			keyboard_bot->control_reset();
		}

		keyboard_bot->control_update();
	}

	return CallNextHookEx(hHook, nCode, wParam, lParam);
}


bot_ardrone_keyboard::bot_ardrone_keyboard(bot_ardrone *b)
{
	keyboard_bot = b;

	hHook = SetWindowsHookEx( WH_KEYBOARD_LL, LowLevelKeyboardProc, NULL, 0 );
	    if (!hHook) {
        MessageBoxA(NULL, "Unable to SetWindowsHookEx!", "Error!", MB_OK);                                            
    }

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