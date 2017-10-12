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
/** \brief Class para fazer a representação de estrada
 *  \file road_representation.h
 *  \author Ricardo Morais
 *  \date Abril 2013
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

class Road_represeentation
{
	private:
		/* Array de pontos para construir o poligno*/
		cv::Point Pts_poly;
		
		/* Vector com o tipo de linha */
		Eigen::VectorXd type;
	
	public:
		/* construir o array de pontos do poligno */
		void SetPoints(cv::Point Pts)
		{
			
			Pts_poly = Pts;			
		}
		
		/* construir o vector com os tipos de pontos */
		void SetTypes(Eigen::VectorXd tt)
		{
			/* limpar o vector */
			type.setZero();
			/* definir o vector */
			type = tt;
		}
		
		/* obter o vector com os pontos */
		cv::Point GetPoints(void)
		{
			return Pts_poly;
		}
		
		/* obter o vector com os tipos dos pontos */
		Eigen::VectorXd GetTypes(void)
		{
			return type;
		}
};
