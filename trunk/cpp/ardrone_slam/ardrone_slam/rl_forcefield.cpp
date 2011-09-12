/* Copyright 2011  Martijn van der Veen
 * GPL version 2 (or later)
 * For the full notice, see COPYRIGHT.txt
 */
 
#include "rl_forcefield.h"


rl_forcefield::rl_forcefield(double width, double height, double coarse)
{
	// force field setup
	int hor, ver;

	hor = (int) floor((width-0.5*coarse) / coarse);
	ver = (int) floor((height-0.5*coarse) / coarse);

	particle_count = hor * ver;
	particles = new rl_particle[particle_count];

	// set default values for particles and pointers to neighbours
	for (int i=0; i<hor; i++) {
		for (int j=0; j<ver; j++) {
			int curr = j*hor + i;
			particles[curr].loc[0] = 0.5*coarse + i * coarse;
			particles[curr].loc[1] = 0.5*coarse + j * coarse;
			particles[curr].vect[0] = 0;
			particles[curr].vect[1] = 0;
			particles[curr].val = 0;
			particles[curr].top    = (j>0)     ? (&particles[curr-hor]) : NULL;
			particles[curr].right  = (i<hor-1) ? (&particles[curr+1])   : NULL;
			particles[curr].bottom = (j<ver-1) ? (&particles[curr+hor]) : NULL;
			particles[curr].left   = (i>0)     ? (&particles[curr-1])   : NULL;
		}
	}
	
}


rl_forcefield::~rl_forcefield(void)
{
}


rl_particle *rl_forcefield::get_nearest_particle(double loc[2], rl_particle *start) {
	// search for nearest particle for location in 'vect'
	// start with particle 'start'
	double d = dist(loc, start->loc);
	rl_particle *curr = start;
	bool changed = true;
	while (changed) {
		changed = false;
		/*
		printf(" -- curr d: %f (%f %f - %f %f)\n", d, curr->loc[0], curr->loc[1], loc[0], loc[1]);
		if (curr->top != NULL)
			printf(" -- top  d: %f (%f %f - %f %f)\n", dist(loc, curr->top->loc), curr->top->loc[0], curr->top->loc[1], loc[0], loc[1]);
		if (curr->right != NULL)
			printf(" -- rgt  d: %f (%f %f - %f %f)\n", dist(loc, curr->right->loc), curr->right->loc[0], curr->right->loc[1], loc[0], loc[1]);
		if (curr->bottom != NULL)
			printf(" -- bot  d: %f (%f %f - %f %f)\n", dist(loc, curr->bottom->loc), curr->bottom->loc[0], curr->bottom->loc[1], loc[0], loc[1]);
		if (curr->left != NULL)
			printf(" -- lft  d: %f (%f %f - %f %f)\n", dist(loc, curr->left->loc), curr->left->loc[0], curr->left->loc[1], loc[0], loc[1]);
		*/

		if (curr->top != NULL && dist(loc, curr->top->loc) < d) {
			d = dist(loc, curr->top->loc);
			curr = curr->top;
			changed = true;
		} else if (curr->right != NULL && dist(loc, curr->right->loc) < d) {
			d = dist(loc, curr->right->loc);
			curr = curr->right;
			changed = true;
		} else if (curr->bottom != NULL && dist(loc, curr->bottom->loc) < d) {
			d = dist(loc, curr->bottom->loc);
			curr = curr->bottom;
			changed = true;
		} else if (curr->left != NULL && dist(loc, curr->left->loc) < d) {
			d = dist(loc, curr->left->loc);
			curr = curr->left;
			changed = true;
		}
	}

	return curr;
}


// after stage transition we need to find the corresponding particle
rl_particle *rl_forcefield::get_matching_particle(double x, double y)
{
	// find horizontal match
	rl_particle *curr = &particles[0];
	while(curr->loc[0] < x) {
		 curr = curr->right;
	}
	// find vertical match
	while(curr->loc[1] < y) {
		curr = curr->bottom;
	}
	return curr;
}

// setting force fields
double rl_forcefield::fdv(double distance, double maximum)
{
	return 0.5 * pow(2, 1.0 - 5.0*distance/maximum);
}

// wall
void rl_forcefield::add_rect_forces(double loc1[2], double loc2[2], double pwr, double range)
{
	double distance;
	double x, y;
	double force[2];
	double scalar;
	for (int i=0; i<particle_count; i++) {
		x = particles[i].loc[0];
		y = particles[i].loc[1];
		distance = 0;
		if (abs(x - loc1[0]) < range) {
			// left
			distance = abs(x - loc1[0]);
			force[0] = 1;
			force[1] = 0;
			scalar = pwr * fdv(distance, range);
			force[0] *= scalar; force[1] *= scalar;
			add(particles[i].vect, force, particles[i].vect); // add and save
		}
		if (abs(x - loc2[0]) < range) {
			// right
			distance = abs(x - loc2[0]);
			force[0] = -1;
			force[1] = 0;
			scalar = pwr * fdv(distance, range);
			force[0] *= scalar; force[1] *= scalar;
			add(particles[i].vect, force, particles[i].vect); // add and save
		}
		if (abs(y - loc1[1]) < range) {
			// top
			distance = abs(y - loc1[1]);
			force[0] = 0;
			force[1] = 1;
			scalar = pwr * fdv(distance, range);
			force[0] *= scalar; force[1] *= scalar;
			add(particles[i].vect, force, particles[i].vect); // add and save
		}
		if (abs(y - loc2[1]) < range) {
			// bottom
			distance = abs(y - loc2[1]);
			force[0] = 0;
			force[1] = -1;
			scalar = pwr * fdv(distance, range);
			force[0] *= scalar; force[1] *= scalar;
			add(particles[i].vect, force, particles[i].vect); // add and save
		}
	}
}

// pylon
void rl_forcefield::add_circle_forces(double loc[2], double radius, double pwr, double range)
{
	double distance;
	double force[2];
	double scalar;
	for (int i=0; i<particle_count; i++) {
		distance = dist(particles[i].loc, loc);
		if (distance - radius < range) {
			scalar = pwr * fdv(distance, range - radius);
			diff(particles[i].loc, loc, force);
			unit(force, force);
			force[0] *= scalar; force[1] *= scalar;
			add(particles[i].vect, force, particles[i].vect); // add and save
		}
	}
}

// pylon spiral forces
void rl_forcefield::add_spiral_forces(double loc[2], double radius, double pwr, double range, double angle)
{
	double distance;
	double force[2];
	double scalar;
	for (int i=0; i<particle_count; i++) {
		distance = dist(particles[i].loc, loc);
		if (distance - radius < range) {
			scalar = pwr * fdv(distance, range - radius);
			diff(particles[i].loc, loc, force);
			rot(force, angle, force);
			unit(force, force);
			force[0] *= scalar;
			force[1] *= scalar;
			add(particles[i].vect, force, particles[i].vect); // add and save
		}
	}

}

// bias towards goal (pylon)
void rl_forcefield::add_attract_forces(double loc[2], double pwr)
{
	double force[2];
	double scalar = pwr;
	for (int i=0; i<particle_count; i++) {
		diff(loc, particles[i].loc, force);
		unit(force, force);
		force[0] *= scalar;
		force[1] *= scalar;
		add(particles[i].vect, force, particles[i].vect);
	}
}
