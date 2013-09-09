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
    }
  } // namespace utils
} // namespace fccl
