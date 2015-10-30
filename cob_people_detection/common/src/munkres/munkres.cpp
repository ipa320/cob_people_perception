/*
 *  munkres.cpp
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

#include "munkres/munkres.h"
#include "limits.h"

munkres::munkres(void)
{
	//default to not showing steps
	diag_on = false;
}

munkres::~munkres(void)
{
}

//Load wieght_array from a vector of vectors of integers
void munkres::load_weights(std::vector<std::vector<int> > x)
{
	//get the row and column sizes of the vector passed in
	int a = x.size(), b = x[0].size();

	//default vectors for populating the matrices
	//these are needed because you need a default object when calling vector::resize()
	std::vector<int> ivector;
	cell default_cell;
	std::vector<cell> default_vector;

	//We need at least as many columns as there are rows

	//If we currently have more rows than columns
	if (a > b)
	{
		//set matrix sizes
		num_rows = b;
		num_columns = a;
		ivector.resize(num_columns, -1);
		weight_array.resize(num_rows, ivector);
		default_vector.resize(num_columns, default_cell);
		cell_array.resize(num_rows, default_vector);

		//populate weight_array and cell_array with the weights from x
		for (int i = 0; i < num_rows; i++)
		{
			for (int j = 0; j < num_columns; j++)
			{
				weight_array[i][j] = x[j][i];
				cell_array[i][j].weight = x[j][i];
			}
		}
	}

	//if the dimensions are correct 
	else
	{
		//set matrix sizes
		num_rows = a;
		num_columns = b;
		ivector.resize(num_columns, -1);
		weight_array.resize(num_rows, ivector);
		default_vector.resize(num_columns, default_cell);
		cell_array.resize(num_rows, default_vector);

		//populate weight_array and cell_array with the weights from x
		for (int i = 0; i < num_rows; i++)
		{
			for (int j = 0; j < num_columns; j++)
			{
				weight_array[i][j] = x[i][j];
				cell_array[i][j].weight = x[i][j];
			}
		}
	}

	//resize our covered and starred vectors
	row_starred.resize(num_rows, false);
	row_cov.resize(num_rows, false);
	column_starred.resize(num_columns, false);
	column_cov.resize(num_columns, false);

	if (diag_on)
	{
		diagnostic(1);
	}

}

//function to copy weight values from cell_array to weight_array	
int munkres::assign(ordered_pair *matching)
{

	//total cost of the matching
	int total_cost = 0;

	//did we find a matching?
	bool matching_found = false;

	//For Checking
	if (diag_on)
	{
		diagnostic(1);
	}

	//try to find a matching
	matching_found = find_a_matching();

	//For Checking
	if (diag_on)
	{
		diagnostic(1);
	}

	//total up the weights from matched vertices
	for (int i = 0; i < num_rows; i++)
	{
		for (int j = 0; j < num_columns; j++)
		{
			if (cell_array[i][j].starred)
			{
				matching[i].col = j;
				matching[i].row = i;
				total_cost += weight_array[i][j];
			}
		}
	}
	return total_cost;
}

//functions to check if there is a star or prime in the 
//current row or column
int munkres::find_star_row(int r)
{

	//check row
	for (int i = 0; i < num_columns; i++)
	{
		//If a starred value is found in current row return true
		if (cell_array[r][i].starred == true)
		{
			row_starred[r] = true;
			column_starred[i] = true;
			return i;
		}
	}
	//If no stars are found return -1
	return -1;
}

int munkres::find_star_column(int c)
{
	//check column
	for (int i = 0; i < num_rows; i++)
	{
		//If a starred value is found in current column return true
		if (cell_array[i][c].starred == true)
		{
			column_starred[c] = true;
			row_starred[i] = true;
			return i;
		}
	}
	//If no stars are found return -1
	return -1;
}

int munkres::find_prime_row(int r)
{

	//check row
	for (int i = 0; i < num_columns; i++)
	{
		//If a primed value is found in current row return 
		//its psoition
		if (cell_array[r][i].primed == true)
		{
			return i;
		}
	}
	//If no primes are found return -1
	return -1;
}

int munkres::find_prime_column(int c)
{
	//check column
	for (int i = 0; i < num_rows; i++)
	{
		//If a primed value is found in current column return 
		//its position
		if (cell_array[i][c].primed == true)
		{
			return i;
		}
	}
	//If no primes are found return -1
	return -1;
}

//The function that will call each of step of Munkres' algorithm in turn
//We're using this since multiple functions use the algorithm
bool munkres::find_a_matching(void)
{
	step1();
	step2();
	return step3();
}

//Function defintions for the steps of Munkres' algorithm
//found at http://csclab.murraystate.edu/bob.pilgrim/445/munkres.html

//We skip step 0 as the matrix is already formed
//Here we subtract the smallest element in each row from the other values in that row
void munkres::step1(void)
{
	//variable to keep track of the smallest value in each row
	int smallest = 0;
	//iterate through rows
	for (int i = 0; i < num_rows; i++)
	{
		//set smallest = first element in the current row
		while (smallest == 0)
		{
			smallest = cell_array[i][0].weight;
			//if the first element is 0 then increase the row
			if (smallest == 0)
			{
				if (i < num_rows - 1)
					i++;
				else
					break;
			}
		}
		//iterate through each value in current row and find the smallest value
		for (int j = 1; j < num_columns; j++)
		{
			//if the current value is a zero, then set smallest to zero
			//and stop searching for a smaller value
			if (cell_array[i][j].weight == 0)
			{
				smallest = 0;
				//break out of the for loop
				j = num_columns;
			}

			//if the current value is smaller than smallest, then
			//set smallest == to current value
			else if (cell_array[i][j].weight < smallest)
			{
				smallest = cell_array[i][j].weight;
			}
		}

		//if the smallest == 0 then we don't need to subtract anything
		//otherwise we need to subtract smallest from everything in the current row
		if (smallest != 0)
		{
			//iterate through the values of current row and subtract
			//smallest from evrything
			for (int j = 0; j < num_columns; j++)
			{
				cell_array[i][j].weight -= smallest;
			}
		}

		//reset smallest for next iteration
		smallest = 0;
	}

	if (diag_on)
	{
		std::cerr << "Step 1" << std::endl;
		diagnostic(2);
	}
}

//Star zeroes that don't have stars (upon thars :P) in the
//same row or column
void munkres::step2(void)
{
	//iterate through rows
	for (int i = 0; i < num_rows; i++)
	{
		//iterate through columns
		for (int j = 0; j < num_columns; j++)
		{
			//if the current index is equal to 0
			if (cell_array[i][j].weight == 0)
			{
				//check for stars in current row
				if (!row_starred[i])
				{
					//if not try to find one
					find_star_row(i);
				}

				//check for stars in current column
				if (!column_starred[j])
				{
					//if not try to find one
					find_star_column(j);
				}

				//if no stars in column or row then star current index
				if (!row_starred[i] && !column_starred[j])
				{
					//star index
					cell_array[i][j].starred = true;
					//mark row as having a star
					row_starred[i] = true;
					//mark column as having a star
					column_starred[j] = true;
				}
			}
		}
	}

	if (diag_on)
	{
		std::cerr << "Step 2" << std::endl;
		diagnostic(3);
	}
}

//cover all columns with starred zeros
//if (num_rows) columns are covered then return true
//to signify that we're done
bool munkres::step3(void)
{
	//an iterator for our while loop
	int iter = 0;

	//loop through columns
	for (int i = 0; i < num_columns; i++)
	{
		//if the column is starred
		if (column_starred[i])
		{
			//cover it
			column_cov[i] = true;
		}
	}
	//while every column so far is covered
	for (int i = 0; i < num_columns; i++)
	{
		if (column_cov[i])
		{
			iter++;
		}
	}

	if (diag_on)
	{
		std::cerr << "Step 3" << std::endl;
		diagnostic(6);
	}

	//if all the rows were covered
	if (iter == num_rows)
	{
		//exit algorithm
		return true;
	}

	//else goto step 4
	else
		return step4();
}

// Find a noncovered zero and prime it.  If there is no starred zero in the row containing this primed zero, Go to Step 5.
// Otherwise, cover this row and uncover the column containing the starred zero. Continue in this manner until there are no
// uncovered zeros left. Save the smallest uncovered value and Go to Step 6.

// probably wrong algorithm
////Find a noncovered zero and prime it
////if there isn't a starred zero in its row then goto step 5
////if there is then cover the current row and uncover the column with the starred zero
////then look for more uncovered zeros
////if there are no uncovered zeros we go to step 6
bool munkres::step4(void)
{
	while (true)
	{
		// find an uncovered zero
		int row = -1;
		int col = -1;
		for (int i = 0; i < num_rows; i++)
		{
			for (int j = 0; j < num_columns; j++)
			{
				if (cell_array[i][j].weight == 0 && row_cov[i] == false && column_cov[j] == false)
				{
					row = i;
					col = j;
					break;
				}
			}
			if (row != -1)
				break;
		}

		// there is no further uncovered zero
		if (row == -1)
		{
			if (diag_on)
			{
				std::cerr << "Step 4" << std::endl;
				diagnostic(6);
			}
			return step6();
		}
		// prime the uncovered zero
		else
		{
			cell_array[row][col].primed = true;

			// If there is no starred zero in the row containing this primed zero, Go to Step 5
			if (row_starred[row] == false)
			{
				if (diag_on)
				{
					std::cerr << "Step 4: " << row << ",  " << col << std::endl;
					diagnostic(6);
				}
				return step5(row, col);
			}
			// Otherwise, cover this row and uncover the column containing the starred zero.
			else
			{
				row_cov[row] = true;
				int starredZeroCol = 0;
				for (starredZeroCol = 0; starredZeroCol < num_rows; starredZeroCol++)
					if (cell_array[row][starredZeroCol].starred == true)
						break;
				column_cov[starredZeroCol] = false;
			}
		}
	}

	return false;

	// old algorithm
	////To find the smallest uncovered value
	//int smallest = 0;

	////iterate through rows
	//for (int i = 0; i < num_rows; i++)
	//{
	//	//if the current row isn't covered
	//	if (!row_cov[i])
	//	{

	//		//set smallest = first element in the current row
	//		while (smallest == 0){
	//			smallest = cell_array[i][0].weight;
	//			//if the first element is 0 then increase the row
	//			if (smallest == 0)
	//			{
	//				if (i < num_rows-1)
	//					i++;
	//				else
	//					break;
	//			}
	//		}

	//		//iterate through columns
	//		for (int j = 0; j < num_columns; j++)
	//		{
	//			//if the column and row aren't covered, the current index is zero,
	//			//and there isn't a star in the current row,
	//			//then prime the current index and go to step 5
	//			if (!column_cov[j] && !row_cov[i] && cell_array[i][j].weight == 0 && !row_starred[i])
	//			{
	//				//prime current index
	//				cell_array[i][j].primed = true;

	//				//if a primed zero with no star in the row exists
	//				//goto step 5
	//				if (diag_on)
	//				{
	//				std::cerr << "Step 4: " << i << ",  " << j <<std::endl;
	//				diagnostic(6);
	//				}
	//				return step5(i, j);


	//			}

	//			//if the column and row aren't covered, the current index is zero,
	//			//and there is a star in the current row,
	//			//then prime the current index, cover the current row,
	//			//and uncover the column with the starred zero
	//			//also reset indeces to 0 to look for zeros that may have been uncovered
	//			else if (!column_cov[j] && !row_cov[i] && cell_array[i][j].weight == 0)
	//			{
	//				//prime current index
	//				cell_array[i][j].primed = true;
	//				//cover current row
	//				row_cov[i] = true;
	//				//uncover column with starred zero
	//				column_cov[find_star_row(i)] = false;
	//				i = 0;
	//				j = 0;
	//			}

	//			//if the column isn't covered, the current index isn't zero,
	//			//and the current index is smaller than smallest so far,
	//			//then set smallest == current index
	//			else if (!column_cov[j] && cell_array[i][j].weight != 0 && cell_array[i][j].weight < smallest)
	//			{
	//				//set smallest == current index
	//				smallest = cell_array[i][j].weight;
	//			}
	//		}
	//	}
	//}

	//if (diag_on)
	//{
	//	std::cerr << "Step 4" << std::endl;
	//	diagnostic(6);
	//}

	//if we don't go to step 5 then go to step 6
	//return step6(smallest);
}

//start at the primed zero found in step 4
//star this zero, erase its prime, and check for a starred zero in current column
//if there is one, erase its star and go to the primed zero in its row
//continue until we arrive at a primed zero with no starred zero in its column
//uncover every row and column in the matrix and return to step 3

//This step checks to see if there is a matching in the current matrix without altering
//any values
bool munkres::step5(int r, int c)
{
	//to determine if we're done creating the sequence
	//of starred and primed zeros
	bool done = false;

	//are we looking for a star or prime
	bool looking_for_star = true;

	//for stupid special case in which we would look for a star in 
	//the current column after starring the current index
	//this returns the current index without looking further down
	//the column, resulting in an error
	int a;

	//create our sequence
	while (!done)
	{
		switch (looking_for_star)
		{

		//if we're looking for a star
		case true:

			//special case protection
			a = r;

			//if there isn't a starred zero in the current column
			if (!column_starred[c])
			{
				//unprime current index
				cell_array[r][c].primed = false;
				//star current index
				cell_array[r][c].starred = true;
				//mark current row starred
				row_starred[r] = true;
				//set done to true
				done = true;
			}
			else
			{
				//set the next row to search to the location of the
				//starred zero in current column
				r = find_star_column(c);
				//unprime current index
				cell_array[a][c].primed = false;
				//star current index
				cell_array[a][c].starred = true;
				//mark current row starred
				row_starred[a] = true;
				//set case to look for prime next
				looking_for_star = false;
			}

			//we can't do this earlier due to needing to check for stars in the column
			//mark the column as starred
			column_starred[c] = true;
			break;

			//if we're looking for a prime
		case false:

			//prime current index
			cell_array[r][c].primed = false;
			//unstar current index
			cell_array[r][c].starred = false;

			//unmark current row as starred
			row_starred[r] = false;

			//set the next column to search to the location of the
			//primed zero in current row
			c = find_prime_row(r);

			//set case to look for star next
			looking_for_star = true;
			break;
		}
	}

	//erase all primes and uncover all rows
	for (int i = 0; i < num_rows; i++)
	{
		for (int j = 0; j < num_columns; j++)
		{
			cell_array[i][j].primed = false;
		}
		row_cov[i] = false;
	}

	//uncover all columns
	for (int i = 0; i < num_columns; i++)
		column_cov[i] = false;

	if (diag_on)
	{
		std::cerr << "Step 5" << std::endl;
		diagnostic(6);
	}
	//go back to step 3
	return step3();
}

// Add the value found in Step 4 to every element of each covered row, and subtract it from every element of each uncovered column.  Return to Step 4 without altering any stars, primes, or covered lines.

// maybe wrong algorithm:
////Subtract the smallest uncovered value found in step 4 from all
////uncovered values and add it to all values whose row AND column
////are covered.  Then return to step 4.
//
////This gives us a new set of zeros to work with since a matching wasn't available
////with the previous set
bool munkres::step6()
{
	// find minimum uncovered value
	int minVal = INT_MAX;
	for (int i = 0; i < num_rows; i++)
		for (int j = 0; j < num_columns; j++)
			if (row_cov[i] == false && column_cov[j] == false)
				if (minVal > cell_array[i][j].weight)
					minVal = cell_array[i][j].weight;

	//iterate through rows
	for (int i = 0; i < num_rows; i++)
	{
		//iterate through columns
		for (int j = 0; j < num_columns; j++)
		{
			// Substract the value found in Step 4 from every element of each uncovered column
			if (column_cov[j] == false)
			{
				//substract sub from its weight
				cell_array[i][j].weight -= minVal;
			}

			// Add the value found in Step 4 to every element of each covered row
			else if (row_cov[i] == true)
				cell_array[i][j].weight += minVal;
		}
	}

	if (diag_on)
	{
		std::cerr << "Step 6" << std::endl;
		diagnostic(6);
	}
	//go back to step 4
	return step4();
}

//Diagnostics only
//Does not affect any results
void munkres::diagnostic(int a) const
{
	switch (a)
	{
	//Show base weights in weight_array
	case 1:
		std::cerr << std::endl << "Base Weights" << std::endl;
		for (int i = 0; i < num_rows; i++)
		{
			for (int j = 0; j < num_columns; j++)
			{
				std::cerr << weight_array[i][j] << " | ";
			}
			std::cerr << std::endl;
		}
		std::cerr << std::endl;
		break;

		//show current weight values of cell_array
	case 2:
		std::cerr << std::endl << "Current Weights" << std::endl;
		for (int i = 0; i < num_rows; i++)
		{
			for (int j = 0; j < num_columns; j++)
			{
				std::cerr << cell_array[i][j].weight << " | ";
			}
			std::cerr << std::endl;
		}
		std::cerr << std::endl;
		break;

		//Show current star placement
	case 3:
		std::cerr << std::endl << "Starred values" << std::endl;
		for (int i = 0; i < num_rows; i++)
		{
			for (int j = 0; j < num_columns; j++)
			{
				if (cell_array[i][j].starred == true)
				{
					std::cerr << cell_array[i][j].weight << "* |  ";
				}
				else
				{
					std::cerr << cell_array[i][j].weight << "  |  ";
				}
			}
			std::cerr << std::endl;
		}
		std::cerr << std::endl;
		break;

		//Show current star placement, covered rows, and covered columns
	case 4:
		std::cerr << std::endl << "Starred values and Lines" << std::endl;
		for (int i = 0; i < num_rows; i++)
		{
			for (int j = 0; j < num_columns; j++)
			{
				if (cell_array[i][j].starred == true)
				{
					std::cerr << cell_array[i][j].weight << "* |  ";
				}
				else
				{
					std::cerr << cell_array[i][j].weight << "  |  ";
				}
			}

			if (row_cov[i])
			{
				std::cerr << "  X";
			}
			std::cerr << std::endl;
		}

		for (int i = 0; i < num_columns; i++)
		{
			if (column_cov[i])
			{

				std::cerr << "X  |  ";
			}
			else
			{
				std::cerr << "   |  ";
			}
		}
		std::cerr << std::endl;
		break;

		//Show current prime placement, covered lines and covered columns
	case 5:
		std::cerr << std::endl << "Primed values and Lines" << std::endl;
		for (int i = 0; i < num_rows; i++)
		{
			for (int j = 0; j < num_columns; j++)
			{
				if (cell_array[i][j].primed == true)
				{
					std::cerr << cell_array[i][j].weight << "' |  ";
				}
				else
				{
					std::cerr << cell_array[i][j].weight << "  |  ";
				}
			}

			if (row_cov[i])
			{
				std::cerr << "  X";
			}
			std::cerr << std::endl;
		}

		for (int i = 0; i < num_columns; i++)
		{
			if (column_cov[i])
			{

				std::cerr << "X  |  ";
			}
			else
			{
				std::cerr << "   |  ";
			}
		}
		std::cerr << std::endl;
		break;

		//Show current star and prime placement, covered rows, and covered columns
	case 6:
		std::cerr << std::endl << "Starred values and Lines" << std::endl;
		for (int i = 0; i < num_rows; i++)
		{
			for (int j = 0; j < num_columns; j++)
			{
				if (cell_array[i][j].starred == true)
				{
					std::cerr << cell_array[i][j].weight << "* |  ";
				}
				else if (cell_array[i][j].primed == true)
				{
					std::cerr << cell_array[i][j].weight << "' |  ";
				}
				else
				{
					std::cerr << cell_array[i][j].weight << "  |  ";
				}
			}

			if (row_cov[i])
			{
				std::cerr << "  X";
			}

			else
			{
				std::cerr << "   ";
			}
			if (row_starred[i])
			{
				std::cerr << "  *";
			}
			std::cerr << std::endl;
		}

		for (int i = 0; i < num_columns; i++)
		{
			if (column_cov[i])
			{

				std::cerr << "X  |  ";
			}
			else
			{
				std::cerr << "   |  ";
			}
		}
		std::cerr << std::endl;

		for (int i = 0; i < num_columns; i++)
		{
			if (column_starred[i])
			{

				std::cerr << "*  |  ";
			}
			else
			{
				std::cerr << "   |  ";
			}
		}
		std::cerr << std::endl;
		break;

	default:
		break;
	}
}
