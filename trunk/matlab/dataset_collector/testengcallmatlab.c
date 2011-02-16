/*************************************************************************************
 *
 * MATLAB (R) is a trademark of The Mathworks (R) Corporation
 *
 * eng file:    testengcallmatlab.c
 * Programmer:  James Tursa
 * Version:     1.0
 * Date:        November 12, 2007
 * Copyright:   (c) 2007 by James Tursa
 * Permission:  Permission is granted to freely distribute and use this code as long
 *              as the header information is included.
 *
 * testengcallmatlab.c is a test diriver program for the engCallMATLAB function that
 * is provided in the engCallMATLAB.c file. This driver program calculates the
 * results of the function call [a,b,c] = cart2pol(3,5,7). The results are converted
 * to double variables and printed to the console, and they are also put into the
 * MATLAB engine workspace as the variables a, b, and c. The engine is intentionally
 * left running after this progrm ends so that you can examine the results directly
 * in the MATLAB engine. Then the engCallMATLAB function is called again with the
 * function kart2pol, an intentional misspelling of the original name. This will
 * result in a return code a 1 indicating a failure.
 *
 * To compile testengcallmatlab.c with the built in lcc compiler, you can use this
 * m-file for win32 (filename is compile_lcc.m):
 *
 *  % Compiles a C main source file. Assumes engine routines are used.
 *  % lcc compiler
 *  function compile_lcc(name)
 *  options = [matlabroot '\bin\win32\mexopts\lccengmatopts.bat'];
 *  disp(['Options = ' options]);
 *  mex('-f', options, name, '-v');
 *  end
 *
 * and invoke it with:
 *
 *  >> compile_lcc('testengcallmatlab.c')
 *
 * This should produce an exe file named testengcallmatlab.exe
 *
 * You can execute this at the MATLAB prompt (assuming that you have installed
 * MATLAB as a console) as follows:
 *
 *  >> !testengcallmatlab
 *
 * You should see the following output on the console:
 *
 *  testengcallmatlab driver program for engCallMATLAB 
 *  calls the function cart2pol(3,5,7) 
 *  engCallMATLAB return code = 0 
 *  a = 1.030377 
 *  b = 5.830952 
 *  c = 7.000000 
 *  Intentionally calling with misspelled name kart2pol 
 *  engCallMATLAB return code = 1 
 *
 * And you can examine the variables directly in the MATLAB engine workspace.
 *
 ******************************************************************************/

#include <stdio.h>
#include <string.h>

#include "engine.h"
#include "matrix.h"
#include "engCallMATLAB.c"

int main( void )
{
    Engine *ep;
    mxArray *plhs[3], *prhs[3];
    int nrhs = 3;
    int nlhs = 3;
    int result;
    char name[] = "cart2pol";
    double a, b, c;
    
    printf("testengcallmatlab driver program for engCallMATLAB\n");
    printf("calls the function cart2pol(3,5,7)\n");
    ep = engOpen( NULL );
    if( ep == NULL )
    {
        printf("Engine did not open.\n");
        printf("One possible reason is MATLAB is not a registered server.\n");
        printf("From Windows, open a Command Prompt and enter:\n");
        printf(">matlab /regserver\n");
        return 1;
    }
    prhs[0] = mxCreateDoubleScalar( 3.0 );
    prhs[1] = mxCreateDoubleScalar( 5.0 );
    prhs[2] = mxCreateDoubleScalar( 7.0 );
    result = engCallMATLAB( ep, nlhs, plhs, nrhs, prhs, name );
    printf( "engCallMATLAB return code = %d\n", result );
    if( result != 0 )
    {
        printf( "engCallMATLAB did not work ... contact the author\n");
        return 1;
    }
    a = mxGetScalar( plhs[0] );
    b = mxGetScalar( plhs[1] );
    c = mxGetScalar( plhs[2] );
    printf( "a = %f\n", a );
    printf( "b = %f\n", b );
    printf( "c = %f\n", c );
    engPutVariable( ep, "a", plhs[0] );
    engPutVariable( ep, "b", plhs[1] );
    engPutVariable( ep, "c", plhs[2] );
    mxDestroyArray( plhs[0] );
    mxDestroyArray( plhs[1] );
    mxDestroyArray( plhs[2] );
    printf( "Intentionally calling with misspelled name kart2pol\n" );
    name[0] = 'k'; // Intentionally misspell function name
    result = engCallMATLAB( ep, nlhs, plhs, nrhs, prhs, name );
    printf( "engCallMATLAB return code = %d\n", result );
    mxDestroyArray( prhs[0] );
    mxDestroyArray( prhs[1] );
    mxDestroyArray( prhs[2] );
    return 0;
// Intentionally did not call engClose so that engine stays active after this
// program ends. You can check the results by comparing a, b, and c in the
// MATLAB engine workspace with the results of [aa,bb,cc]=cart2pol(3,5,7)
}
