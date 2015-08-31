/* 
 * File:   TEncOpenCL.h
 * Author: augusto
 *
 * Created on April 16, 2015, 10:02 AM
 */

#ifndef TENCOPENCL_H
#define	TENCOPENCL_H

#include "TLibCommon/TypeDef.h"
#include "TLibCommon/TComMv.h"
#include "CL/cl.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <math.h>
#include <assert.h>

class TEncOpenCL {
    
protected:
    
    // ====================================================================================================================
// Variable to configure OpenCL
// ====================================================================================================================
    cl_int                  errNum;                 ///< Indicate error 
    cl_uint                 numPlatforms;           ///< Number of platforms found in host
    cl_uint                 numDevices;             ///< Number of devices found that support OpenCL
    cl_platform_id*         platformIDs;            ///< Information of platforms found
    cl_device_id*           deviceIDs;              ///< Information of Devices found
    cl_context              context;                ///< Create OpenCL context 
    cl_command_queue        queue;                  ///< Create OpenCL Command Queue
    cl_program              program;                ///< Create OpenCL Program
    cl_kernel               kernelCalc;             ///< Create Kernel to calculate SAD
    cl_kernel               kernelCompare;          ///< Create Kernel to compare SAD
    std::size_t             paramValueSize;         ///< Info Device
    Bool                    deviceFound ;           ///< Flag that indicates if a OpenCL Device was found
    Bool                    compileKernel;          ///< Flag that indicates if Kernel was compiled
    Int                     deviceId;               ///< Device Id
    Char*                   info;                   ///< Info of OpenCL Device
    const Char*             kernelSource;           ///< kernel Source Filename
    Bool                    enabled;
   
	
// ====================================================================================================================
// Buffer definition
// ====================================================================================================================
    
    // ========= Input Buffers ====================
    cl_mem                  pelCtuBuffer;           ///< Global Buffer of Pel CTU 
    cl_mem                  pelAreaBuffer;          ///< Global Buffer of Pel Search Area
        
    // ============== Output Buffers =================
   
    cl_mem                  sadBuffer;              ///< minimum SAD Buffer
    cl_mem                  XarrayBuffer;           ///< pos X of minimum SAD Buffer
    cl_mem                  YarrayBuffer;           ///< pos Y of minimum SAD Buffer
    cl_mem                  ruiCostBuffer;          ///< Rui Cost of Blocks Buffer
    cl_mem                  tempSadBuffer;
       
    // ========== Auxiliar Buffers=====================
   
    
    // ======== Input Data =======================
   
    Pel*                    pelCtuArray;            ///< Pointer to pixel of initial position of CTU   
    Pel*                    pelSearchArray;         ///< Pointer to pixel of initial position of Search area
    cl_int                  iRefStride;             ///< Reference image stride
    cl_int                  iCtuStride;             ///< Ctu Stride
    
    // ========== Auxiliars Data ===================
    Pel*                    srchAreaPtr;         
    
    
    // ========= Output Arrays =========================
    Int*                    Xarray;                 ///< Pos X of minimun SAD blocks
    Int*                    Yarray;                 ///< Pos X of minimun SAD blocks
    Distortion*             minSad;                 ///< Minimum SAD blocks
    Distortion*             ruiCosts;                ///< Rui Cost of Blocks
    
    // ========= Values =========================

    cl_uint                 maxCtuWidth;
    cl_uint                 maxCtuHeight;
    cl_int                  searchRange;
    cl_int                  areaSize; 
    UInt                    m_lambda;
 
    //Sub-function to display error
    inline Bool checkError(cl_int err, const char * name)
    {
        Bool success = true;
        if (err != CL_SUCCESS) {
            fprintf(stderr, "ERROR: %s ( %d )\n", name, err );
            success = false;
        }
        return success;
    }
    
    

public:
    TEncOpenCL();
    virtual         ~TEncOpenCL();
    Bool            compileKernelSource(const Char* fileName, const Char* kernelNameCalc);
    Bool            findDevice ( Int device); 
    Bool            createBuffers ( UInt i_maxCtuWidth, UInt i_maxCtuHeight, Int i_searchRange);
    Void            calcMotionVectors(Pel* pelCtu, Pel* pelSearch, Int i_iRefStride, Int i_iCtuStride, Int i_areaSize, TComMv* pcMvSrchRngLT);
    
    //======== getters and setters ================
    Int             getDeviceId         ()              { return deviceId; }
    Void            setDeviceId         ( Int i )       { deviceId = i; }
    const Char*     getDeviceInfo       ()              { return info; }
    Distortion*     getRuiCost          ()              { return ruiCosts; }      
    Int*            getX                ()              { return Xarray; }
    Int*            getY                ()              { return Yarray; }
    /* Lambda for calculate motion cost - Don't use cu_transquant_bypass */
    Void            setLambda           (Double lambda) { m_lambda =  (UInt)floor(65536.0 * sqrt(lambda)); }
    Void            setEnabled          (Bool e)        { enabled = e; }

 
protected:  
    Void             xFillSADBuffer();
    Void             xResetArrays();
};

#endif	/* TENCOPENCL_H */
