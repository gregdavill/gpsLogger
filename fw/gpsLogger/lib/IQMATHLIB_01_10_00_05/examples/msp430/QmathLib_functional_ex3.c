/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//*****************************************************************************
// QmathLib_functional_ex3: Qmath functional example.
//
// This example demonstrates how to use the Qmath functions to perform basic
// operations.
//
// B. Peterson
// Texas Instruments Inc.
// May 2014
// Built with CCS version 6.0.0.00190 and IAR Embedded Workbench version 6.10.1
//*****************************************************************************
#include "msp430.h"

#include <stdint.h>

#define PI      3.1415926536

/* Select the global Q value and include the Qmath header file. */
#define GLOBAL_Q    12
#include "QmathLib.h"

volatile float res;       	// floating point variable to verify results

int main(void)
{
    _q qA, qB, qC;          // Q variables using global type
    _q8 q8A, q8B, q8C;      // Q variables using Q8 type
    _q15 q15A, q15C;  		// Q variables using Q15 type
    
    /* Disable WDT. */
    WDTCTL = WDTPW + WDTHOLD;
    
    /* Basic global Q operations. */
    qA = _Q(1.0);
    qB = _Q(2.5);
    qC = qA + qB; res=_QtoF(qC);                // 3.5 = 1.0 + 2.5
    qC = qC - _Qmpy2(qA); res=_QtoF(qC);        // 1.5 = 3.5 - 2*(1.0)
    qC = _Qmpy(qB, qC); res=_QtoF(qC);          // 3.75 = 2.5 * 1.5
    qC = _Qdiv(qC, qB); res=_QtoF(qC);          // 1.5 = 3.75 / 2.5
    qC = _Qsqrt(qB); res=_QtoF(qC);             // 1.58114 = sqrt(2.5)
    
    /* Trigonometric global Q operations. */
    qA = _Q(PI/4.0);
    qB = _Q(0.5);
    qC = _Qsin(qA); res=_QtoF(qC);              // 0.70710 = sin(PI/4)
    qC = _Qcos(qA); res=_QtoF(qC);              // 0.70710 = cos(PI/4)
    qC = _Qatan(qB); res=_QtoF(qC);             // 0.46365 = atan(0.5)
    
    /* Exponential global Q operations. */
    qA = _Q(2.71828);
    qB = _Q(0.5);
    qC = _Qlog(qA); res=_QtoF(qC);              // 1.0 = ln(e)
    qC = _Qexp(qB); res=_QtoF(qC);              // 1.6487 = e^0.5
    
    /* Basic explicit type Q8 operations. */
    q8A = _Q8(1.0);
    q8B = _Q8(2.5);
    q8C = q8A + q8B; res=_Q8toF(q8C);           // 3.5 = 1.0 + 2.5
    q8C = q8C - _Qmpy2(q8A); res=_Q8toF(q8C);   // 1.5 = 3.5 - 2*(1.0)
    q8C = _Q8mpy(q8B, q8C); res=_Q8toF(q8C);    // 3.75 = 2.5 * 1.5
    q8C = _Q8div(q8C, q8B); res=_Q8toF(q8C);    // 1.5 = 3.75 / 2.5
    q8C = _Q8sqrt(q8B); res=_Q8toF(q8C);        // 1.58114 = sqrt(2.5)
    
    /* Trigonometric explicit type Q15 operations. */
    q15A = _Q15(PI/4.0);
    q15C = _Q15sin(q15A); res=_Q15toF(q15C);    // 0.70710 = sin(PI/4)
    q15C = _Q15cos(q15A); res=_Q15toF(q15C);    // 0.70710 = cos(PI/4)
    
    /* Explicit type Q8 to Global Q conversion with saturation check. */
    q8A = _Q8(1.0);
    q8B = _Q8(16.0);
    qC = _Q8toQ(_Qsat(q8A, _QtoQ8(MAX_Q_POS), _QtoQ8(MAX_Q_NEG)));
    res = _QtoF(qC);    // _Q8(1.0) -> _Q(1.0) (q8A does not saturate)
    qC = _Q8toQ(_Qsat(q8B, _QtoQ8(MAX_Q_POS), _QtoQ8(MAX_Q_NEG)));  
    res = _QtoF(qC);    // _Q8(16.0) -> ~MAX_Q_POS (q8A saturates to maximum positive _Q value)

    return 0;
}
