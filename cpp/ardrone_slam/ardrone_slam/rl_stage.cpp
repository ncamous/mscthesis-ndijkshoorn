/* Copyright 2011  Martijn van der Veen
 * GPL version 2 (or later)
 * For the full notice, see COPYRIGHT.txt
 */

#include "rl_stage.h"


rl_stage::rl_stage(reinflearn *rl)
{
	this->rl = rl;
	this->tr = new rl_transition;
	this->ff = new rl_forcefield(rl->layout->width, rl->layout->height, rl->layout->coarse);
}


rl_stage::~rl_stage(void)
{
}

// there is just one stage transition
void rl_stage::add_stage_transition(double v1[2], double v2[2], rl_stage *stage) {
	cp(v1, tr->loc1);
	cp(v2, tr->loc2);
	tr->stage = stage;
}

void rl_stage::save_to_file(char *filename, double loc[2], rl_particle *part_last, double vect_fly[2]) {
	/* Format:
	 * - current location
	 * - current force vector
	 * - particle per line:
	 *   x y vect_x vect_y val
	 */
	CopyFile(filename, "../../particles.txt.bak", false);
	ofstream fp;
	fp.open(filename);
	fp << loc[0] << " " << loc[1] << endl;
	fp << part_last->loc[0] << " " << part_last->loc[1] << " " << vect_fly[0]/8 << " " << vect_fly[1]/8 << endl;
	for (int i=0; i<ff->particle_count; i++) {
		fp << ff->particles[i].loc[0] << " ";
		fp << ff->particles[i].loc[1] << " ";
		fp << ff->particles[i].vect[0] << " ";
		fp << ff->particles[i].vect[1] << " ";
		fp << ff->particles[i].val;
		fp << endl;
	}
	fp.close();
}


void rl_stage::save_field_to_file(char *filename, char *tmp) {
	/* Format:
	 * - particle per line:
	 *   x y vect_x vect_y val
	 */
	ofstream fp;
	fp.open(tmp);
	for (int i=0; i<ff->particle_count; i++) {
		fp << ff->particles[i].loc[0] << " ";
		fp << ff->particles[i].loc[1] << " ";
		fp << ff->particles[i].vect[0] << " ";
		fp << ff->particles[i].vect[1] << " ";
		fp << ff->particles[i].val;
		fp << endl;
	}
	fp.close();
	CopyFile(tmp, filename, false);

}

void rl_stage::load_field_from_file(char *file) {
	double x, y, vx, vy, val;
	x = y = vx = vy = val = 0;
	int count = 0;
	rl_particle *part;
	string line;
	ifstream fp (file);
	if (fp.is_open()) {
		while (fp.good() && count < ff->particle_count) {
			part = &(ff->particles[count++]);

			getline(fp, line);
			sscanf(line.data(), "%lf %lf %lf %lf %lf", &x, &y, &vx, &vy, &val);

			//part = ff->get_matching_particle(x, y);
			part->vect[0] = vx;
			part->vect[1] = vy;
			part->val = val;
		}
	} else {
		printf("Unable to open file %s.\n", file);
	}
}


bool rl_stage::check_for_stage_transition(double v1[2], double v2[2])
{
	double *a = tr->loc1;
	double *b = v2;
	double *c = tr->loc2;
	double *d = v1;
	if (sign(a, b, c) == sign(b, c, d) 
	 && sign(b, c, d) == sign(c, d, a)
	 && sign(c, d, a) == sign(d, a, b)) {
		printf("a = %f %f\n", a[0], a[1]);
		printf("b = %f %f\n", b[0], b[1]);
		printf("c = %f %f\n", c[0], c[1]);
		printf("d = %f %f\n", d[0], d[1]);

		cout << sign(a, b, c) << sign(b, c, d) << sign(c, d, a) << sign(d, a, b);
		return true;
	} else {
		return false;
	}
}
