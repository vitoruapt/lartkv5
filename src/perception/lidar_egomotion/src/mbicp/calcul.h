/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:

  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************/
/***************************************************/
/* Last Revised: 
$Id: calcul.h 8465 2009-12-16 00:44:13Z gbiggs $
*/
/***************************************************/
/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/**
\file
\brief Auxiliary function declaration for the MbICP module
*/

#ifndef Calcul
#define Calcul

#include <stdio.h>
#include <math.h>
#include "TData.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 
   Este fichero tiene operaciones de transformacion de sistemas de referencia, 
   transformaciones de puntos entre sistemas, de paso de coordenadadas polares,
   a cartesianas y de corte de segmentos

*/

/* --------------------------------------------------------------------------------------- */
/* TRANSFORMACIONES DE PUNTO DE UN SISTEMA DE REFERENCIA A OTRO                            */
/* --------------------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------------------- */
/* transfor_directa_p                                                                      */ 
/*  .... Hace la transformacion directa de un punto a un sistema a otro                    */ 
/*  .... In: (x,y) las coordenadas del punto, sistema es el sistema de referencia          */
/*  .... Out: en sol se devuelve las coordenadas del punto en el nuevo sistema             */

void transfor_directa_p ( float x, float y, Tsc *sistema, Tpf *sol );

/* --------------------------------------------------------------------------------------- */
/* transfor_directa_p                                                                      */ 
/*  .... Hace la transformacion directa de un punto a un sistema a otro                    */ 
/*  .... La diferencia es que aqui el punto de entrada es el (0,0) (optimiza la anterior)  */
/*  .... In: (x,y) las coordenadas del punto, sistema es el sistema de referencia          */
/*  .... Out: en sol se devuelve las coordenadas del punto en el nuevo sistema             */

void transfor_directa_pt0(float x, float y, 
			  Tsc *sistema, Tpf *sol);
  
/* --------------------------------------------------------------------------------------- */
/* transfor_inversa_p                                                                      */ 
/*  .... Hace la transformacion inversa de un punto a un sistema a otro                    */ 
/*  .... In: (x,y) las coordenadas del punto, sistema es el sistema de referencia          */
/*  .... Out: en sol se devuelve las coordenadas del punto en el nuevo sistema             */

void transfor_inversa_p ( float x, float y, Tsc *sistema, Tpf *sol );

/* --------------------------------------------------------------------------------------- */
/* TRANSFORMACIONES DE COMPOSICION E INVERSION DE SISTEMAS DE REFERENCIA                   */
/* --------------------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------------------- */
/* composicion_sis                                                                         */ 
/*  .... Realiza la composicion de sistemas de referencia en otro sistema                  */ 
/*  .... In: compone sis1 y sis2                                                           */
/*  .... Out: la salida sisOut es el resultado de la composicion de los sistemas           */
/*  .... Nota: resulta muy importante el orden de las entradas en la composicion           */

void composicion_sis(Tsc *sis1,Tsc *sis2,Tsc *sisOut);

/* --------------------------------------------------------------------------------------- */
/* inversion_sis                                                                           */ 
/*  .... Realiza la inversion de un sistema de referencia                                  */ 
/*  .... In: sisIn es el sistema a invertir                                                */
/*  .... Out: sisOut es el sistema invertido                                               */

void inversion_sis(Tsc *sisIn, Tsc *sisOut);

/* --------------------------------------------------------------------------------------- */
/* TRANSFORMACIONES DE PUNTO DE UN SISTEMA DE REFERENCIA A OTRO                            */
/* --------------------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------------------- */
/* car2pol                                                                                 */ 
/*  .... Transforma un punto de coordenadas cartesianas a polares                          */ 
/*  .... In: el punto en coordenadas cartesianas a transformar                             */
/*  .... Out: el punto salida en coordenadas polares                                       */

void car2pol(Tpf *in, Tpfp *out);

/* --------------------------------------------------------------------------------------- */
/* pol2car                                                                                 */ 
/*  .... Transforma un punto de coordenadas polares a cartesianas                          */ 
/*  .... In: el punto entrada en coordenadas polares a transformar                         */
/*  .... Out: el punto en coordenadas cartesianas transformado                             */

void pol2car(Tpfp *in, Tpf *out);

/* --------------------------------------------------------------------------------------- */
/* TRANSFORMACIONES DE PUNTO DE UN SISTEMA DE REFERENCIA A OTRO                            */
/* --------------------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------------------- */
/* corte_segmentos                                                                         */ 
/*  .... Calcula el punto de corte entre dos segmentos                                     */ 
/*  .... In: las coordenadas de los puntos extremos (x1,y1)-(x2,y2) y (x3,y3)-(x4,y4)      */
/*  .... Out: sol son las coordenadas del punto de corte. return --> 1 si hay corte. -->0 no */

int corte_segmentos ( float x1, float y1, float x2, float y2, 
		      float x3, float y3, float x4, float y4,
		      Tpf *sol );


/* Normaliza el angulo entre [-PI, PI] */
float NormalizarPI(float ang);

#ifdef __cplusplus
}
#endif

#endif 
