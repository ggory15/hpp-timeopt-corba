#include <hpp/corbaserver/fwd.hh>
#include <hpp/corbaserver/common.hh>

namespace hpp{
    namespace corbaServer{
        vector3_t floatSeqToVector3 (const floatSeq& dofArray){
            vector3_t result;
            for (unsigned int iDof=0; iDof < 3; ++iDof) {
                result [iDof] = dofArray [iDof];
            }
            return result;
        }
       vector_t floatSeqToVector (const floatSeq& dofArray, const size_type expectedSize = -1){           
            size_type inputDim = (size_type)dofArray.length();
            vector_t result (inputDim);
            for (size_type iDof=0; iDof < inputDim; ++iDof) {
                result [iDof] = dofArray [(CORBA::ULong)iDof];
            }
            return result;   
       }
    }
}