/* Copyright 2011  Martijn van der Veen
 * GPL version 2 (or later)
 * For the full notice, see COPYRIGHT.txt
 */

#pragma once
#include <windows.h>
#include <fstream>
#include <iostream>
#include <string>

#include "linalg.h"
#include "rl_forcefield.h"
#include "rl_stage.h"

class bot_ardrone;
struct bot_ardrone_measurement;
struct bot_ardrone_control;
struct bot_ardrone_frame;

using namespace std;

static bot_ardrone *rl_bot;


// structures
typedef struct rl_particle {
	double loc[2];
	double vect[2];
	double val;
	struct rl_particle *top;
	struct rl_particle *right;
	struct rl_particle *bottom;
	struct rl_particle *left;
} rl_particle;

typedef struct rl_history {
	struct rl_particle *particle;
	bool explore;
	double explore_vect[2];
	double speed[2];
} rl_history;

typedef struct rl_layout {
	double width;
	double height;
	double offset[2];
	double pylon1[2];
	double pylon2[2];
	double pylonr;
	double ardroner;
	double coarse;
	double speed_x;
	double speed_y;
	// reinf learn:
	int history;
	double epsilon; // probability for exploration
	double alpha;   // learning rate
	double gamma;   // discount factor
} rl_layout;

typedef struct rl_transition {
	double loc1[2];
	double loc2[2];
	class rl_stage *stage;
} rl_transition;


typedef struct rl_stats {
	char *filename;
	long last_transition;
	long number_of_transitions;
} rl_stats;


class reinflearn
{
public:
	// vars
	rl_stage *stage_curr;
	rl_layout *layout;
	rl_stats *stats;
	rl_history *history; // TODO: hard-coded for now
	
	// construct/destruct
	reinflearn(bot_ardrone *bot);
	~reinflearn(void);

	// field calculations
	void real2sim(double vect_in[2], double vect_out[2]);
	void sim2real(double vect_in[2], double vect_out[2]);

	// control robot
	void control_set(int type, double velocity);
	void fly_vector(double vect[2]);

	// history
	void history_clear();
	void add_to_history(struct rl_particle *particle, double dir_curr[2], bool explore);

	// collisions
	bool collision(double loc_curr[2]);
	void move_away_from_obstacle(double loc_curr[2], double vect_fly[2]);

	// helper functions
	long timestamp_millis();
	void save_stats_to_file(struct rl_stats *stats);
	
	// reinforcement learning
	void value_update(double loc_curr[2], double loc_last[2]);
	void vector_update();
	double reward(double loc_curr[2], double loc_last[2]);
	
};