/* Copyright 2011  Martijn van der Veen
 * GPL version 2 (or later)
 * For the full notice, see COPYRIGHT.txt
 */

#include "global.h"
#include "reinflearn.h"
#include "bot_ardrone.h"
#include <io.h>
#include <math.h>

#include <cv.hpp>



reinflearn::reinflearn(bot_ardrone *bot)
{
	printf("REINF STARTED\n");

	rl_bot = bot;

	// define level constants
	layout = new rl_layout;
	layout->width  = 2 * 9;
	layout->height = 2 * 4;
	layout->offset[0] = -9; // will be added to input location
	layout->offset[1] = -4; // - 1.5;
	layout->pylon1[0] = -5.1;
	layout->pylon1[1] = 0;
	layout->pylon2[0] = 4.5;
	layout->pylon2[1] = 0;
	layout->pylonr = 0.45;
	layout->ardroner = 0.45;
	layout->coarse = 0.6;
	layout->speed_x = 0.11;
	layout->speed_y = 0.11;
	layout->history = 5;
	layout->epsilon = 0.0;
	layout->alpha   = 0.3;
	layout->gamma   = 0.9;

	bool restore = false;
	bool learn   = false;
	
	stats = new rl_stats;
	stats->filename = "../../stats.txt";
	stats->last_transition = 0;
	stats->number_of_transitions = 0;

	history = new rl_history[layout->history];
	history_clear();

	//Sleep(3000);

	// Create stages
	rl_stage *stage_l2r = new rl_stage(this);
	rl_stage *stage_r2l = new rl_stage(this);

	double v1[2], v2[2]; // v1: pylon, v2: wall
	real2sim(layout->pylon1, v1);
    v2[0] = 0;
	v2[1] = v1[1];
	stage_r2l->add_stage_transition(v1, v2, stage_l2r);
	real2sim(layout->pylon2, v1);
    v2[0] = layout->width;
	v2[1] = v1[1];
	stage_l2r->add_stage_transition(v1, v2, stage_r2l);
	stage_curr = stage_l2r;


	if (restore) {
		
		// restore force fields from files
		stage_r2l->load_field_from_file("../../stage_r2l.txt");
		stage_l2r->load_field_from_file("../../stage_l2r.txt");

	} else {
	
		// Set initial force fields
		
		// - wall
		v1[0] = 0;
		v1[1] = 0;
		v2[0] = layout->width;
		v2[1] = layout->height;
		stage_l2r->ff->add_rect_forces(v1, v2, 0.8, 5);
		stage_r2l->ff->add_rect_forces(v1, v2, 0.8, 5);
		// - pylon1
		real2sim(layout->pylon1, v1);
		stage_l2r->ff->add_circle_forces(v1, layout->pylonr, 0.7, 10);
		stage_r2l->ff->add_circle_forces(v1, layout->pylonr, 0.7, 10);
		stage_l2r->ff->add_spiral_forces(v1, layout->pylonr, 0.6, 10, PI/2);
		stage_r2l->ff->add_spiral_forces(v1, layout->pylonr, 0.6, 10, PI/2);
		// - pylon2
		real2sim(layout->pylon2, v1);
		stage_l2r->ff->add_circle_forces(v1, layout->pylonr, 0.7, 10);
		stage_r2l->ff->add_circle_forces(v1, layout->pylonr, 0.7, 10);
		stage_l2r->ff->add_spiral_forces(v1, layout->pylonr, 0.6, 10, -PI/2);
		stage_r2l->ff->add_spiral_forces(v1, layout->pylonr, 0.6, 10, -PI/2);
		// attract
		real2sim(layout->pylon1, v1);
		stage_r2l->ff->add_attract_forces(v1, 0.25);
		real2sim(layout->pylon2, v1);
		stage_l2r->ff->add_attract_forces(v1, 0.25);

	}



	// Learning algorithm

	double vect_fly[2];
	double loc_curr[2];
	float loc_curr_f[2];
	double loc_last[2];
	double dir_curr[2];
	bool update_with_reward = false;
	long time_now;
	long time_last = 0;
	//bot_ardrone_measurement *m = new bot_ardrone_measurement;
	rl_particle *part_last = &(stage_curr->ff->particles[0]);

	Sleep(500);

	stats->last_transition = (long) time(NULL);

	while (1) {

		if (stop_behavior)
			break;


		//bot->recorder->get_last(m);
		bot->get_slam_pos(loc_curr_f);

		loc_curr[0] = (double) loc_curr_f[0] / 1000.0;
		loc_curr[1] = (double) loc_curr_f[1] / 1000.0;
		real2sim(loc_curr, loc_curr);
		time_now = timestamp_millis();


		part_last = stage_curr->ff->get_nearest_particle(loc_curr, part_last);


		if ((history[0].particle != NULL && part_last->loc[0] != history[0].particle->loc[0])
		  || (history[0].particle != NULL && part_last->loc[1] != history[0].particle->loc[1])
		  || reward(loc_curr, loc_last) != 0) {
			
			if (learn && (this->history[1].particle != NULL)) {

				if (reward(loc_curr, loc_last) != 0)
					update_with_reward = true; // only once per particle + reward

				// reinforcement learning
				value_update(loc_curr, loc_last);
				vector_update();

			}

		}


		// calculate new fly command and do RL
		if ( (this->history[0].particle == NULL)
		  || (part_last->loc[0] != history[0].particle->loc[0])
		  || (part_last->loc[1] != history[0].particle->loc[1]) ) {


			diff(loc_curr, loc_last, dir_curr);
			//unit(dir_curr, dir_curr);
			double scal = 2; // ten opzichte van fly_vector commando
			scal *= (double)(time_now - time_last)/1000;
			dir_curr[0] *= scal;
			dir_curr[1] *= scal;
			// TODO: now uses unit vector (direction only)


			// new particle, determine if we will explore or use policy
			// but for collision, move outwards
			//printf("loc: %f %f -> ", loc_curr[0], loc_curr[1]);
			if (collision(loc_curr)) {
				// ugly fix, for now
				//move_away_from_obstacle(loc_curr, vect_fly);
				printf("Obstacle! %f %f\n", vect_fly[0], vect_fly[1]);
			} /*else*/ if ((double)rand()/RAND_MAX < layout->epsilon) {
				add_to_history(part_last, dir_curr, true);
				cp(history[0].explore_vect, vect_fly);
				printf("Explore! %f %f\n", vect_fly[0], vect_fly[1]);
			} else {
				add_to_history(part_last, dir_curr, false);
				cp(history[0].particle->vect, vect_fly);
				printf("Force:   %f %f\n", vect_fly[0], vect_fly[1]);
			}

			update_with_reward = false;

		}
		// else: same particle, thus same command as last time



		
		
		// do some book keeping after transition
		if (time_last != 0) {
			if (stage_curr->check_for_stage_transition(loc_curr, loc_last)) {
				// stage transition
				printf("Stage transition! %p -> %p\n", stage_curr, stage_curr->tr->stage);
				stage_curr = stage_curr->tr->stage;
				part_last = stage_curr->ff->get_matching_particle(part_last->loc[0], part_last->loc[1]);
				// update stats
				save_stats_to_file(stats);
				stats->last_transition = (long) time(NULL);
				stats->number_of_transitions++;
				history_clear();
			}
		}

		
		// save for visualisation
		stage_curr->save_to_file("../../particles.txt", loc_curr, part_last, vect_fly);
		// save for restore
		stage_r2l->save_field_to_file("../../stage_r2l.txt", "../../stage_r2l_tmp.txt");
		stage_l2r->save_field_to_file("../../stage_l2r.txt", "../../stage_l2r_tmp.txt");


		// fly command
		unit(vect_fly, vect_fly); // constant speed
		vect_fly[0] *= layout->speed_x;
		vect_fly[1] *= layout->speed_y;
		fly_vector(vect_fly);

		cp(loc_curr, loc_last);
		time_last = timestamp_millis();


		/*
		printf("loc: %f, %f; part: %f, %f; fly: %f %f \n",
			   loc_curr[0],
			   loc_curr[1],
			   part_last->loc[0],
			   part_last->loc[1],
			   vect_fly[0],
			   vect_fly[1]);
		/* */

		Sleep(30); // improves speed estimation
	}

}

reinflearn::~reinflearn()
{
}

void reinflearn::control_set(int type, double velocity) {

	if (rl_bot->control_get(BOT_ARDRONE_Velocity, type) != velocity)
	{
		rl_bot->control_set(BOT_ARDRONE_Velocity, type, (float)velocity);
		rl_bot->control_update();
	}

}

void reinflearn::fly_vector(double vect[2]) {
	float or[3];

	rl_bot->get_slam_or(or);

	// vect[0]: linear (to front/back); vect[1]: lateral (to right/left)

	// rotate absolute vector 'vect' back gt_or
	//bot_ardrone_measurement *m = new bot_ardrone_measurement;
	//rl_bot->recorder->get_last(m);

	rot(vect, -1*or[2], vect);
	// give command to robot
	rl_bot->control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LinearVelocity, (float)vect[0]);
	rl_bot->control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LateralVelocity, (float)vect[1]);
	rl_bot->control_update();

}

// from unreal-level to this simulation
void reinflearn::real2sim(double vect_in[2], double vect_out[2]) {
	vect_out[0] = vect_in[0] - layout->offset[0];
	vect_out[1] = vect_in[1] - layout->offset[1];
}

// from this simulation to unreal-level
void reinflearn::sim2real(double vect_in[2], double vect_out[2]) {
	vect_out[0] = vect_in[0] + layout->offset[0];
	vect_out[1] = vect_in[1] + layout->offset[1];
}


// history
void reinflearn::history_clear() {
	for (int i=0; i<layout->history; i++) {
		history[i].particle = NULL;
		history[i].explore_vect[0] = 0;
		history[i].explore_vect[1] = 0;
		history[i].speed[0] = 0;
		history[i].speed[1] = 0;
	}
}

// adds pointers to original particles to the history
// will make random explore vector when necessary
void reinflearn::add_to_history(struct rl_particle *particle, double dir_curr[2], bool explore) {
	// move older particles
	for (int i=layout->history - 2; i>0; i--) {
		if (history[i-1].particle == NULL)
			continue;
		history[i].explore = history[i-1].explore;
		cp(history[i-1].explore_vect, history[i].explore_vect);
		history[i].particle = history[i-1].particle;
		cp(history[i-1].speed, history[i].speed);
	}
	// add particle
	history[0].particle = particle;
	history[0].explore = explore;
	if (explore) {
		double weight = 2;
		double expl[2];
		expl[0] = ((double)rand()/RAND_MAX - 0.5);
		expl[1] = ((double)rand()/RAND_MAX - 0.5);
		unit(expl, expl);
		expl[0] *= weight;
		expl[1] *= weight;

		history[0].explore_vect[0] = expl[0];
		history[0].explore_vect[1] = expl[1];
	} else {
		history[0].explore_vect[0] = 0;
		history[0].explore_vect[1] = 0;
	}
	cp(dir_curr, history[0].speed);
}


// collisions
bool reinflearn::collision(double loc_curr[2]) {
	double x = loc_curr[0];
	double y = loc_curr[1];
	double r = layout->ardroner;
	double p1[2], p2[2];
	real2sim(layout->pylon1, p1);
	real2sim(layout->pylon2, p2);

	if (dist(loc_curr, p1) < layout->pylonr + r) {
		// collision with pylon1
		return true;
	}

	if (dist(loc_curr, p2) < layout->pylonr + r) {
		// collision with pylon2
		return true;
	}

	if (x < r || x > layout->width - r || y < r || y > layout->height - r) {
		// collision with wall
		return true;
	}

	// no collision
	return false;
}

void reinflearn::move_away_from_obstacle(double loc_curr[2], double vect_fly[2]) {
	double x = loc_curr[0];
	double y = loc_curr[1];
	double r = layout->ardroner;
	double p1[2], p2[2];
	real2sim(layout->pylon1, p1);
	real2sim(layout->pylon2, p2);

	// no collision:
	vect_fly[0] = 0;
	vect_fly[1] = 0;

	// collision with pylon1
	if (dist(loc_curr, p1) < layout->pylonr + r) {
		diff(loc_curr, p1, vect_fly);
		unit(vect_fly, vect_fly);
	} else if (dist(loc_curr, p2) < layout->pylonr + r) {
		// collision with pylon2
		diff(loc_curr, p2, vect_fly);
		unit(vect_fly, vect_fly);
	} else {
		if (x < r) {
			// collision with wall
			vect_fly[0] += 1;
			vect_fly[1] += 0;
		}
		if (x > layout->width - r) {
			// collision with wall
			vect_fly[0] += -1;
			vect_fly[1] += 0;
		}
		if (y < r) {
			// collision with wall
			vect_fly[0] += 0;
			vect_fly[1] += 1;
		}
		if (y > layout->height - r) {
			// collision with wall
			vect_fly[0] += 0;
			vect_fly[1] += -1;
		}
		if (norm(vect_fly) > 0) {
			unit(vect_fly, vect_fly);
		}
	}
}


// Helper functions
long reinflearn::timestamp_millis() {
	return (long)GetTickCount();

	//SYSTEMTIME *tm;
	//GetSystemTime(tm);
	//return (24*60*1000*(int)tm->wDay + 60*1000*(int)tm->wHour + 1000 * (int)tm->wSecond + (int)tm->wMilliseconds);
}

void reinflearn::save_stats_to_file(rl_stats *stats) {
	long diff = (long)time(NULL) - stats->last_transition;
	
	fstream fp;
	fp.open(stats->filename, fstream::in | fstream::out | fstream::app);
	fp << stats->number_of_transitions << " " << diff << endl;
	fp.close();
}


// reinforcement learning
void reinflearn::value_update(double loc_curr[2], double loc_last[2]) {

	double val_last, val_curr, val_new, r;

	// update last (with reward)
	val_last = history[1].particle->val;
	val_curr = history[0].particle->val;
	r = reward(loc_curr, loc_last);

	val_new = val_last + layout->alpha * (r + layout->gamma * val_curr - val_last);

	history[1].particle->val = val_new;

	// update rest of history (using gamma and prev. value)
	for (int h=2; h < layout->history; h++) {

		if (history[h].particle == NULL)
			break;
		
		val_last = history[h].particle->val;
		val_curr = layout->gamma * val_curr;
		
		val_new = val_last + layout->alpha * (layout->gamma * val_curr - val_last);

		history[h].particle->val = val_new;
	
	}

}

void reinflearn::vector_update() {

	double vect_last[2], vect_new[2], speed_last[2];
	double lambda, val_curr, val_last;

	
	for (int h = layout->history - 1; h>0; h--) {

		if (history[h].particle == NULL)
			continue;

		cp(history[h].speed, speed_last);
		//unit(speed_last, speed_last);
		cp(history[h].particle->vect, vect_last);
		val_curr = history[0].particle->val;
		val_last = history[h].particle->val;
	
		lambda = layout->alpha * (val_curr - val_last) / h;

		vect_new[0] = (1 - lambda) * vect_last[0] + lambda * speed_last[0];
		vect_new[1] = (1 - lambda) * vect_last[1] + lambda * speed_last[1];

		//printf("from %f %f ", history[1].particle->vect[0], history[1].particle->vect[1]);
		if (norm(vect_new) > 1.5) {
			printf("Vector long, shortening..\n");
			double nrm = norm(vect_new);
			vect_new[0] = vect_new[0] * 2.0 / nrm;
			vect_new[1] = vect_new[1] * 2.0 / nrm;
		} else {
			cp(vect_new, history[h].particle->vect);
		}
		//printf("to %f %f\n", history[1].particle->vect[0], history[1].particle->vect[1]);
	
	}

}

double reinflearn::reward(double loc_curr[2], double loc_last[2]) {
	
	if (stage_curr->check_for_stage_transition(loc_curr, loc_last)) {
		return 1;
	}

	if (collision(loc_curr)) {
		printf("COLLISION\n");
		return -1;
	}

	return 0;

}
