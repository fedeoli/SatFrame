/*  F=trapzf(X,Y) computes the integral of Y with respect to X using
    the trapezoidal method. X and Y must be vectors of the same length.
    For trapzf to work properly:
    X must be a STRICTLY monotonically increasing vector.

	Warning: not much in the way of error checking, since this slows
             things down, so pay attention to the argument passed to the function!!!

    function trapzf(), V. 1.2 Umberto Picchini, October 2006.
    Report bugs or suggestions to umberto.picchini@biomatematica.it


    Installation:
    --------------

    Simply copy the trapzf.dll file in a directory recognizable by MATLAB. You won't
    need trapzf.c to perform computations, but it is useful if you want to
    read the user instructions or to customize your code.
    If you modify trapzf.c then you must run the following command

    mex trapzf.c

    in order to obtain the new dll file.

Ex1:
>> x=[1:1:1000];
>> y=log(sqrt(x+1.001)-1.001);
>> area=trapzf(x,y)
   area =
    2.8933e+003

Ex2:
>> x=[1:.000001:2];
>> y=log(sqrt(x+1.001)-1.001);
>> tic; area=trapz(x,y); eval1=toc;  % the MATLAB standard trapezoidal integrator
>> eval1

eval1 =

    0.2800

>> tic; area=trapzf(x,y); eval2=toc;
>> eval2

eval2 =

    0.0200


*/


#include "mex.h"

/* Input arguments */
#define X_data      prhs[0]
#define Y_data      prhs[1]


/* Output arguments */
#define out         plhs[0]

static double trapzf(double *area, double *xv, double *yv, int N)
{
  int i;
  double h;

  if(N==1)   // trapzf(x,y) returns zero for a scalar x
    {*area = 0;
      return *area;
    }

  h = xv[1] - xv[0]; /* the integration stepsize */ 
  for(i=1; i<N-1; i++)
  {*area = *area + yv[i];
  }
  *area = h/2 * (yv[0] + 2*(*area) + yv[N-1]);

  return *area;
} 


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
int N, N1, N2;
double *area;
double *xv, *yv ;

/* Find the length of the X array */
   N1 = mxGetNumberOfElements(X_data); 
/* Find the length of the Y array */
   N2 = mxGetNumberOfElements(Y_data);
if(N1!=N2)
   mexErrMsgTxt("Input x and y must have the same length.");
else 
   N = N1;

/* Create an mxArray of real values for the output */
   out = mxCreateDoubleMatrix(1,1,mxREAL);
/* Get the data passed in */
   xv       = mxGetPr(X_data);
   yv       = mxGetPr(Y_data);
/* the output */
   area     = mxGetPr(out);
/* Do the actual computations in a subroutine */
   trapzf(area,xv,yv,N);
   return;
}