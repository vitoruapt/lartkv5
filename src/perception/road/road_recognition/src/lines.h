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
/** \brief Class fazer a converção de coordenadas
 *  \file lines.h
 *  \author Ricardo Morais
 *  \date Abril 2013
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

using namespace std;
using namespace cv;
using namespace Eigen;
class Lines
{
	public:
		vector<Vec4i> end_points;
		vector<double> rhos;
		vector<double> thetas;
		MatrixXi extreme_points;
	
		/* Converter as linhas encontradas de coordenadas cartesianas para polares */
		void calculatePolarCoodinates(void)
		{
			rhos.clear();
			thetas.clear();
			
			for(uint i=0 ; i<end_points.size() ; i++)
			{
				double x1 = end_points[i][0];
				double y1 = end_points[i][1];
				double x2 = end_points[i][2];
				double y2 = end_points[i][3];
				
				double th = atan2(x1-x2,y2-y1);
				double rho = x1*cos(th) + y1*sin(th);
// 				cout<<"th="<<th<<" rho="<<rho<<endl;
				
				rhos.push_back(rho);
				thetas.push_back(th);
			}
		}

		/* Retirar os valores fora da escala */
		void extractInterval(double rho_max,double rho_min,double theta_max,double theta_min)
		{
			vector<Vec4i> end_points_local;
			vector<double> rhos_local;
			vector<double> thetas_local;
	
			/* Verificar se os valores de rho e theta estao dentro do intervalo pretendido */
			for(uint i=0;i<end_points.size();i++)
			{
				if(thetas[i]>theta_min && thetas[i]<theta_max)
				{
					end_points_local.push_back(end_points[i]);
					rhos_local.push_back(rhos[i]);
					thetas_local.push_back(thetas[i]);
				}
		
			}
			
			end_points=end_points_local;
			rhos=rhos_local;
			thetas=thetas_local;
		}
		
		/* Calcular os pontos extremos das linhas encontradas */
		MatrixXi extremePoints(int rows , int cols)
		{
			VectorXi P1(2);
			VectorXi P2(2);
			extreme_points.resize(20,4);
			extreme_points.setOnes();
			
			for (uint i=0; i<20 ; i++)
				extreme_points(i,3) = 2;
			
			for (uint i=0 ; i<end_points.size() ; i++)
			{
				/* Pontos extremos do segmento de recta econtrado */
				P1 << end_points[i][0] , end_points[i][1];
				P2 << end_points[i][2] , end_points[i][3];
				
				/* Variáveis auxiliares */
				double Px,Py;
				long double k;
				
				/* Pontos dos extremos da imagem */
				VectorXi l1(2);
				l1(0) = 0;
				l1(1) = 0;
				
				VectorXi l2(2);
				l2(0) = 0;
				l2(1) = cols;
				
				VectorXi l3(2);
				l3(0) = cols;
				l3(1) = rows;
				
				VectorXi l4(2);
				l4(0) = 0;
				l4(1) = rows;
				
		
				/* intersecção com as linhas horizontais */
				if (thetas[i]!=0 && thetas[i]!=PI)
				{
					/* Calculo da intersecção com a linha 4*/
					Py = rows;
					k = ( (double)Py-(double)P1(1) )/( (double)P2(1)-(double)P1(1) );
					Px = P1(0) + k*(P2(0)-P1(0));
					
					if (Px<=cols && Px>=0) /* Verificar se cumpre os limites da img */
					{
						extreme_points(i,0) = Px;
						extreme_points(i,1) = Py;
					}
					
					/* Calculo da intersecção com a linha 2*/
					Py = 1;
					k = ( (double)Py-(double)P1(1) )/( (double)P2(1)-(double)P1(1) );
					Px = P1(0) + k*(P2(0)-P1(0));
					
					if (Px<=cols && Px>=0) /* Verificar se cumpre os limites da img */
					{
						extreme_points(i,0) = Px;
						extreme_points(i,1) = Py;
					}
				}
				
				/* intersecção com as linhas verticais */
				if (thetas[i]!=(PI/2) && thetas[i]!=(-PI/2) )
				{
					/* Calculo da intersecção com a linha 1*/
					Px = 1;
					k = ( (double)Px-(double)P1(0) )/( (double)P2(0)-(double)P1(0) );
					Py = P1(1) + k*(P2(1)-P1(1));
					if (Py>0 && Py<rows) /* Verificar se cumpre os limites da img */
					{
						extreme_points(i,2) = Px;
						extreme_points(i,3) = Py;
					}
					/* Calculo da intersecção com a linha 3*/
					Px = cols;
					k = ( (double)Px-(double)P1(0) )/( (double)P2(0)-(double)P1(0) );
					Py = P1(1) + k*(P2(1)-P1(1));
					
					if (Py>0 && Py<rows) /* Verificar se cumpre os limites da img */
					{
						extreme_points(i,2) = Px;
						extreme_points(i,3) = Py;
					}
				}
				
			}
// 			cout<<"------------------------"<<endl;
			
			return extreme_points;
		}
		
};
