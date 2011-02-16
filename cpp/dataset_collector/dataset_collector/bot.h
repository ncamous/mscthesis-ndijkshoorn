#pragma once

#include "botinterface.h"


class bot
{
public:
	bot(botinterface *i);
	~bot(void);
	void bot::set(int opt, float val);
	void bot::reset();

	botinterface *i;

protected:
	void bot::update();
};