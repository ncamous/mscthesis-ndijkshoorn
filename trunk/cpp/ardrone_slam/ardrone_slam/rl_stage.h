/* Copyright 2011  Martijn van der Veen
 * GPL version 2 (or later)
 * For the full notice, see COPYRIGHT.txt
 */

#pragma once

#include "reinflearn.h"

class rl_stage
{
public:
	// vars
	class reinflearn *rl;
	class rl_forcefield *ff;
	struct rl_transition *tr;
	
	// methods
	rl_stage(reinflearn *rl);
	~rl_stage(void);
	void add_stage_transition(double v1[2], double v2[2], rl_stage *stage);
	bool check_for_stage_transition(double v1[2], double v2[2]);
	
	// visualisations
	void save_to_file(char *filename, double loc[2], struct rl_particle *part_last, double vect_fly[2]);
	// restore
	void save_field_to_file(char *filename, char *tmp);
	void load_field_from_file(char *filename);


};

