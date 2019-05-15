// Author: Alessandro Palleschi
// pchip()

#ifndef PCHIP_H
#define PCHIP_H

#include <Eigen/Core>


double inline sign (double a) {
  if(a==0)
  {
    return 0;
  }
  
  double sgn = (a>0)?1:-1;
  return sgn;
}

void inline pchipslopes(std::vector<double> x, std::vector<double> y, std::vector<double> del,std::vector<double> &d)
{
  double n = x.size();
    double size = y.size();
  d.resize(size);

  if(n==2) {
    for(int i=0;i<size;i++) d.at(i) = del.at(0);
    return;
  }

  std::vector<int> index;
  bool found = false;

  for(int k=0;k<size-2;k++)
  {
    double check = sign(del.at(k))*sign(del.at(k+1));
    if(check > 0){

      index.push_back(k);
      found = true;
    }
  }

  std::vector<double> h;
  h.resize(size-1);
  
  


   // hs = h(k)+h(k+1);
   // w1 = (h(k)+hs)./(3*hs);
   // w2 = (hs+h(k+1))./(3*hs);
   // dmax = max(abs(del(k)), abs(del(k+1)));
   // dmin = min(abs(del(k)), abs(del(k+1)));
   // d(k+1) = dmin./conj(w1.*(del(k)./dmax) + w2.*(del(k+1)./dmax));

  for(int k=0;k<size-1;k++)
  {
    h.at(k) = x.at(k+1)-x.at(k);

  }
    if(found){
    for(int j=0;j<index.size();j++){

    double hs = h.at(index.at(j))+h.at(index.at(j)+1);
    double w1 = (h.at(index.at(j))+hs)/(3*hs);
    double w2 = (h.at(index.at(j)+1)+hs)/(3*hs);
    double dmax = std::max(fabs(del.at(index.at(j))),fabs(del.at(index.at(j)+1)));
    double dmin = std::min(fabs(del.at(index.at(j))),fabs(del.at(index.at(j)+1)));
    d.at(index.at(j)+1) = dmin/(w1*del.at(index.at(j))/dmax + w1*del.at(index.at(j)+1)/dmax);
    }
  }
    
 
   d.at(0) = ((2*h.at(0)+h.at(1))*del.at(0) - h.at(0)*del.at(1))/(h.at(0)+h.at(1));
   if (sign(d.at(0)) != sign(del.at(0)))
   {
      d.at(0) = 0;
   }
   else if ((sign(del.at(0)) != sign(del.at(1))) && (fabs(d.at(0)) > fabs(3*del.at(0))))
   {
      d.at(0) = 3*del.at(0);
   }
   
   d.at(n-1) = ((2*h.at(n-2)+h.at(n-3))*del.at(n-2) - h.at(n-2)*del.at(n-3))/(h.at(n-2)+h.at(n-3));
   if(sign(d.at(n-1)) != sign(del.at(n-2)))
   { 
      d.at(n-1) = 0;
   }
   else if ((sign(del.at(n-2)) != sign(del.at(n-3))) && (fabs(d.at(n-1)) > fabs(3*del.at(n-2))))
   {
      d.at(n-1) = 3*del.at(n-2);
   }

}



void inline pchip(std::vector<double> x, std::vector<double> y, std::vector<double> t,std::vector<double> &v)
{
  double sizey = y.size()-1;
    double sizex = x.size()-1;
    if(sizex!=sizey)
    { 
      ROS_ERROR("Vector Dimensions not consistent");
      return;
    }

  std::vector<double> h;
  h.resize(sizey);
  std::vector<double> del;
  del.resize(sizey);

  for(int k=0;k<sizex;k++)
  {
    h.at(k) = x.at(k+1)-x.at(k);
    del.at(k) = (y.at(k+1)-y.at(k))/h.at(k);

  }



    double m = sizey+1;
    std::vector<double> slopes,dzzdx,dzdxdx;
    pchipslopes(x, y, del, slopes);
    double n = sizex+1;
    double d = 1;
    std::vector<double> dxd = h;
    dzzdx.resize(slopes.size()-1);
    dzdxdx.resize(slopes.size()-1);

    for(int k=0;k<slopes.size()-1;k++){


      dzzdx.at(k) = (del.at(k)-slopes.at(k))/dxd.at(k);
      dzdxdx.at(k) = (slopes.at(k+1)-del.at(k))/dxd.at(k);


    }
    

    int dm1 = (n-1)*d;
    Eigen::MatrixXd coeff(dm1,4);
    
    for(int j=0;j<dm1;j++)
    {
      coeff.row(j) << (dzdxdx.at(j)-dzzdx.at(j))/dxd.at(j), 2*dzzdx.at(j)-dzdxdx.at(j), slopes.at(j),y.at(j);
    }

    int l = 1;
    v.resize(t.size());
    for(int j=0;j<t.size()-1;j++){
      if(t.at(j) < x.at(l)) {
        double t_aux = t.at(j)-x.at(l-1);
        v.at(j) = coeff(l-1,0)*pow(t_aux,3)+coeff(l-1,1)*pow(t_aux,2)+coeff(l-1,2)*pow(t_aux,1)+coeff(l-1,3);

      }
      else{
        l++;
        v.at(j) = coeff(l-1,3);

      } 
  }
        v.at(v.size()-1) = y.at(y.size()-1);


}
#endif