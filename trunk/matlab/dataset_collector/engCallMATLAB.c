/*************************************************************************************
 *
 * MATLAB (R) is a trademark of The Mathworks (R) Corporation
 *
 * eng file:    engCallMATLAB.c
 * Programmer:  James Tursa
 * Version:     1.0
 * Date:        November 12, 2007
 * Copyright:   (c) 2007 by James Tursa
 * Permission:  Permission is granted to freely distribute and use this code as long
 *              as the header information is included.
 *
 * Requires:    #include <stdio.h>
 *              #include <string.h>
 *              #include "engine.h"
 *
 * engCallMATLAB calls a MATLAB function using the MATLAB engine. This is the eng
 * equivalent of the function mexCallMATLAB for mex files. The engine pointer ep
 * must point to an engine that is already opened with the engOpen function.
 *
 * Inputs:  ep   = Engine pointer (Engine *)
 *          nlhs = Number of desired output arguments (int)
 *          nrhs = Number of input arguments (int)
 *          prhs = Array of pointers to input mxArrays (mxArray *[])
 *          name = Name of the function to call (char *)
 *
 * Outputs: plhs = Array of pointers to output mxArrays (mxArray *[])
 *          returns 0 = success, 1 = failure
 *
 * Notes: This routine dynamically allocates memory for the return mxArray variables
 *        pointed to by the plhs array. It is up to the calling routine to destroy
 *        these arrays with the mxDestroyArray routine when done with them.
 *
 *        If engCallMATLAB returns a 1 for failure, then none of the plhs pointers
 *        will be set to anything, and there will be no need to destroy them.
 *
 *        The engCallMATLAB function works as follows:
 *        - Builds a command string with the appropriate number of dummy input
 *          names (ECMI00, ECMI01, ECMI02, ...) and dummy output names (ECMO00,
 *          ECMO01, ECMO02, ...).
 *        - Puts the inputs into the MATLAB workspace under the dummy input names.
 *        - Calls engEvalString to evaluate the command string.
 *        - Gets the resulting dummy outputs from the MATLAB workspace.
 *        - Clears all of the temporary variables (ECMI00, ...) from the MATLAB
 *          workspace.
 *        - If there is an error in any of the above processes, then all of the
 *          temporary mxArray variables are destroyed.
 *
 *        Because the temporary variables are put into the MATLAB workspace, this
 *        routine will only work if you do not have any existing MATLAB workspace
 *        variable names that clash with the dummy names that this routine uses.
 *        These names are:
 *          ECMInn, where nn = any two digit number from 00 - 99
 *          ECMOnn, where nn = any two digit number from 00 - 99
 *
 *        Example: To evaluate the equivalent of the following MATLAB function
 *
 *                   [a,b,c] = cart2pol[3,5,7)
 *
 *                 You would have to have these inputs:
 *
 *                   ep = pointer to Engine already opened with engOpen
 *                   nlhs = 3
 *                   plhs = array at least 3 in size
 *                   nrhs = 3
 *                   prhs[0] = pointer to mxArray containing the value 3
 *                   prhs[1] = pointer to mxArray containing the value 5
 *                   prhs[2] = pointer to mxArray containing the value 7
 *                   name = "cart2pol"
 *
 *                 This routine will build the following command string:
 *
 *                   [ECMO00,ECMO01,ECMO02]=cart2pol(ECMI00,ECMI01,ECMI02);
 *
 *                 After putting ECMI00, ECMI01, and ECMI02 into the MATLAB
 *                 workspace, the engEvalString routine will be called with
 *                 the command string. Then the result variables ECMO00,
 *                 ECMO01, and ECMO02 will be gotten from the MATLAB workspace
 *                 and assigned to the plhs[] array pointers. This routine will
 *                 then clear all of the MATLAB workspace variables for this
 *                 example:
 *
 *                   ECMO00, ECMO01, ECMO02, ECMI00, ECMI01, ECMI02
 *
 ***********************************************************************************/

int engCallMATLAB(Engine *ep, int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[],
                  const char *name)
{
    const int limit = 500; // Length limit of command line
    char command[500]; // Command string to be passed to engEvalString
    char nn[4]; // Temporary string used for number conversion to char
    const char ECMI[] = "ECMI"; // Base name of MATLAB workspace input var
    const char ECMO[] = "ECMO"; // Base name of MATLAB workspace output var
    char ecmname[] = "ECM___"; // Base name used for putting & getting vars
    char clearname[] = "clear ECM____"; // Base command for clearing vars
    int i, k, m; // Loop index
    int cj; // Index into command string as we are building it
    int inx, outx; // Number of input &output variables to clear
    int result; // Return value, 0 = success, 1 = failure
//-----
    
    result = 1; // Default result is failure
    
//\
// Check for proper numbers
///
    
    if( nlhs <= 0 || nrhs < 0 || nlhs > 99 || nrhs > 99 )
        return 1;
    
//\
// Constuct output string
///
    
    if( 7 * nlhs + 2 >= limit )
        return 1;
    
    if( nlhs == 1 ) // only one output variable
    {
        strcpy( command+0, "ECMO00=" );
        cj = 7;
    }
    else // multiple outputs, enclose in brackets, comma separated
    {
        command[0] = '[';
        cj = 1;
        for( i=0; i<nlhs; i++ )
        {
            sprintf( nn, "%3d", 100+i );
            for( k=0; k<4; k++ )
                command[cj++] = ECMO[k];
            command[cj++] = nn[1];
            command[cj++] = nn[2];
            command[cj++] = ',';
        }
        command[cj-1] = ']';
        command[cj++] = '=';
    }
    
//\
// Add the function name
///
    
    if( cj > limit )
        return 1;
    m = 0;
    while( name[m] != '\0' )
    {
        if( cj >= limit )
            return 1;
        command[cj++] = name[m++];
    }
    if( cj >= limit )
        return 1;
    
//\
// Construct input string
///
    
    if( nrhs != 0 )
    {
        if( cj + 7 * nrhs + 2 >= limit )
            return 1;
        command[cj++] = '(';
        for( i=0; i<nrhs; i++ )
        {
            sprintf( nn, "%3d", 100+i );
            for( k=0; k<4; k++ )
                command[cj++] = ECMI[k];
            command[cj++] = nn[1];
            command[cj++] = nn[2];
            command[cj++] = ',';
        }
        command[cj-1] = ')';
    }
    
//\
// Don't print results on MATLAB display, so add semi-colon
///
    
    command[cj] = ';';
    if( ++cj >= limit )
        return 1;
    
//\
// Terminate the command string with null character
///
    
    command[cj] = '\0';
    
//\
// Put the input variables into the MATLAB workspace
///
    
    outx = 0;
    for( i=0; i<nrhs; i++ )
    {
        inx = i;
        sprintf( ecmname+3, "%3d", 100+i );
        ecmname[3] = 'I';
        ecmname[6] = '\0';
        if( engPutVariable( ep, ecmname, prhs[i] ) != 0 )
            goto cleanup;
    }
    inx = nrhs;
    
//\
// Evaluate the function in the MATLAB workspace
///
    
    if( engEvalString( ep, command ) != 0 )
        goto cleanup;
    
//\
// Put the output variables into the MATLAB workspace
///
    
    for( i=0; i<nlhs; i++ )
    {
        outx = i;
        sprintf( ecmname+3, "%3d", 100+i );
        ecmname[3] = 'O';
        ecmname[6] = '\0';
        if( (plhs[i] = engGetVariable( ep, ecmname )) == NULL )
            goto cleanup;
    }
    outx = nlhs;
    
//\
// Success
///
    
    result = 0;
    
//\
// Clean up the MATLAB workspace and temporary mxArray variables
///
    
cleanup:
    while( inx-- )
    {
        sprintf( clearname+9, "%3d", 100+inx );
        clearname[9] = 'I';
        clearname[12] = ';';
        clearname[13] = '\0';
        engEvalString( ep, clearname );
    }
    while( outx-- )
    {
        sprintf( clearname+9, "%3d", 100+outx );
        clearname[9] = 'O';
        clearname[12] = ';';
        clearname[13] = '\0';
        engEvalString( ep, clearname );
        if( result != 0 )
        {
            mxDestroyArray( plhs[outx] );
            plhs[outx] = NULL;
        }
    }

//\
// Return the result, 0 = success, 1 = failure
//
    
    return result;
    
}
