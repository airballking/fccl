#include<fccl/utils/Printing.h>

namespace fccl
{
  namespace utils
  {
    std::ostream& operator<<(std::ostream& out, const KDL::Vector& v)
    {
      out << "[ " << v.x() << " " << v.y() << " " << v.z() << " ]";
      return out;
    }
  
    std::ostream& operator<<(std::ostream& out, const KDL::Rotation& m)
    {
      for(unsigned int i=0; i<3; i++)
      {
        for(unsigned int j=0; j<3; j++)
        {
          out << m(i,j) << " ";
          if(j!=3)
            out << " ";
        }
        if(i!=3)
          out << "\n";
      }
  
      return out;
    }
  
    std::ostream& operator<<(std::ostream& out, const KDL::Frame& f)
    {
      out << "p: " << f.p << "\n";
      out << "M:\n" << f.M;

      return out;
    }

    std::ostream& operator<<(std::ostream& out, const KDL::Twist& t)
    {
      out << "translation: " << t.vel << "\n";
      out << "rotation: " << t.rot;
 
      return out;
    }

    std::ostream& operator<<(std::ostream& out, const Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic >& m)
    {
      for(unsigned int row=0; row<m.rows(); row++)
      {
        for(unsigned int column=0; column<m.cols(); column++)
        {
          out << "  " << m(row, column);
        }
        if(row < (m.rows() - 1))
          out << "\n";
      }  
 
      return out;
    }
  } // namespace utils
} // namespace fccl
