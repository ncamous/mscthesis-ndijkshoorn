#include <stdio.h>

#include "bot.h"
#include "botinterface.h"

bot::bot(botinterface *i)
{
	this->i = i;
	printf("Bot: interface set\n");
}


bot::~bot(void)
{
}


void bot::set(int opt, float val)
{
	printf("not implemented\n");
}


void bot::reset()
{

}


void bot::update()
{
	printf("not implemented\n");
}