/*
 *  munkres.h
 *  Hungaraian Assignment Algorithm
 *
 *  Authored by Ryan Rigdon on [May15].
 *  Copyright 2008 M. Ryan Rigdon
 *	mr.rigdon@gmail.com
 *
 */

//This file is part of The Kuhn-Munkres Assignment Algorithm.
//
//The Kuhn-Munkres Assignment Algorithm is free software: you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
//the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.
//
//The Kuhn-Munkres Assignment Algorithm is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License
//along with Foobar.  If not, see <http://www.gnu.org/licenses/>.


#ifndef MUNKRES_H
#define	MUNKRES_H

#include <iostream>
#include <vector>

//struct for the cells of the matrix
struct cell
{
	//The weight value in the cell
	int weight;
	//Values for whether the cell is primed or starred
	bool starred, primed;

	//initialize starred and primed values to false
	cell()
	{
		starred = false;
		primed = false;
		weight = -1;
	}
};

//It's an ordered_pair, not much else to say
struct ordered_pair
{
	int row, col;
};

class munkres
{
public:
	munkres(void);
	~munkres(void);

	//turn diagnostics on or off
	void set_diag(bool a)
	{
		diag_on = a;
	}

	//assign a matching
	//returns the total weight of the matching
	//takes a pointer to integer as a parameter and this
	//will be the matching in the form of an array
	int assign(ordered_pair *matching);

	//Load weight array from a vector of vectors of integers
	//Accepts an object of type vector< vector<int> >, which is
	//a matrix of any dimensions with integer values > -1
	void load_weights(std::vector<std::vector<int> > x);

private:

	//Delimiters to show number of operable rows and columns
	//after the cell array is populated
	int num_rows;
	int num_columns;

	//arrays to keep track of which columns and rows have 
	//starred zeroes
	std::vector<bool> row_starred, column_starred;

	//array to show which rows and columns are covered
	std::vector<bool> row_cov, column_cov;

	//A boolean value to turn daignostics on or off
	bool diag_on;

	//Initialize all variables for operations
	void init(void);

	//The matrix operated on by Munkres' algorithm
	//(could be better than an array in the future)
	std::vector<std::vector<cell> > cell_array;

	//array to store the weights for calculating total weight
	std::vector<std::vector<int> > weight_array;

	//functions to check if there is a starred zero in the current row or column
	int find_star_column(int c);
	int find_star_row(int r);

	//functions to check if there is a primed zero in the current row or column
	int find_prime_column(int c);
	int find_prime_row(int r);

	//These are the declarations for Munkres' algorithm steps
	void step1(void);
	void step2(void);
	bool step3(void);
	bool step4(void);
	bool step5(int, int);
	bool step6();

	//The function that will call each of step of Munkres' algorithm in turn
	//We're using this since multiple functions use the algorithm
	bool find_a_matching(void);

	//A function simply for diagnostic purposes
	//Useful for testing new code and to help both myself and anyone who
	//wants to modify this in the future
	void diagnostic(int a) const;
};

#endif // MUNKRES_H
