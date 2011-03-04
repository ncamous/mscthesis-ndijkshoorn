#pragma once

#include <string>

using namespace std;

bool usarsim_msgparser_value(string *input, string *key, string *value);
int usarsim_msgparser_type(string *input);

int usarsim_msgparser_int(string *input, char *tag);
double usarsim_msgparser_double(string *input, char *tag);
void usarsim_msgparser_double3(string *input, char *tag, double *d);
float usarsim_msgparser_float(string *input, char *tag);
void usarsim_msgparser_float3(string *input, char *tag, float *d);