/* 
 * File:   TEncOpenCL.cpp
 * Author: augusto
 * 
 * Created on April 16, 2015, 10:02 AM
 */

#include <limits>
#include "TEncOpenCL.h"

#if !defined(CL_CALLBACK)
#define CL_CALLBACK
#endif

void CL_CALLBACK contextCallback(
	const char * errInfo,
	const void * private_info,
	size_t cb,
	void * user_data)
{
	std::cout << "Error occured during context use: " << errInfo << std::endl;
	// should really perform any clearup and so on at this point
	// but for simplicitly just exit.
	exit(1);
}

TEncOpenCL::TEncOpenCL() {
    
    context         = NULL;      
    queue           = NULL;              
    program         = NULL;            
    kernelCalc      = NULL;         
    kernelCompare   = NULL; 
       
}


TEncOpenCL::~TEncOpenCL() {
    
    
    clFinish(queue);
    clReleaseContext(context);
    
    if(enabled)
    {
        errNum = clReleaseMemObject(pelCtuBuffer);
        checkError(errNum, "clReleaseMemObject - Delete Ctu Buffer");

        errNum = clReleaseMemObject(pelAreaBuffer);
        checkError(errNum, "clReleaseMemObject - Delete search area Buffer");

        errNum = clReleaseMemObject(sadBuffer);
        checkError(errNum, "clReleaseMemObject - Delete min sad Buffer");

        errNum = clReleaseMemObject(tempSadBuffer);
        checkError(errNum, "clReleaseMemObject - Delete temp sad Buffer");

        errNum = clReleaseMemObject(XarrayBuffer);
        checkError(errNum, "clReleaseMemObject - Delete x Buffer");

        errNum = clReleaseMemObject(YarrayBuffer);
        checkError(errNum, "clReleaseMemObject - Delete y Buffer");
    }
    Xarray = NULL;
    Yarray = NULL;
    minSad = NULL;
}

Bool TEncOpenCL::findDevice(Int device)
{
    deviceFound = true;
    deviceId = device;
    // Select an OpenCL platform to run on.  
    errNum = clGetPlatformIDs(0, NULL, &numPlatforms);
    deviceFound =  checkError((errNum != CL_SUCCESS) ? errNum : (numPlatforms <= 0 ? -1 : CL_SUCCESS), "No Platforms Found"); 
 
    platformIDs = (cl_platform_id *)alloca(sizeof(cl_platform_id) * numPlatforms);

    errNum = clGetPlatformIDs(numPlatforms, platformIDs, NULL);
    checkError((errNum != CL_SUCCESS) ? errNum : (numPlatforms <= 0 ? -1 : CL_SUCCESS), "No Platforms Found");

    if(deviceFound)
    {
        // Iterate through the list of platforms until we find one that supports
        // a GPU device, otherwise fail with an error.
        deviceIDs = NULL;

        cl_uint i;
        for (i = 0; i < numPlatforms; i++)
        {
            //Find only GPU Devices
            errNum = clGetDeviceIDs(platformIDs[i], CL_DEVICE_TYPE_GPU, 0, NULL, &numDevices);            
            if (errNum != CL_SUCCESS && errNum != CL_DEVICE_NOT_FOUND)
            {
                deviceFound = checkError(errNum, "No devices Found");
                
            }

            //Construct a vector with devices found
            else if (numDevices > 0) 
            {
                deviceIDs = (cl_device_id *)alloca(sizeof(cl_device_id) * numDevices);
                errNum = clGetDeviceIDs(platformIDs[i], CL_DEVICE_TYPE_GPU, numDevices, &deviceIDs[0], NULL);
                deviceFound = checkError(errNum, "No devices found");
                break;
            }
        }
        
        //if found device, display information
        if(deviceFound)
        {
            if(deviceId < 0 || deviceId > (numDevices - 1) )
            {
                deviceId = 0;
                printf("ID device not found, use default GPU device \n");                
            }
            
            //Obtain device vendor name to display info   
            errNum = clGetDeviceInfo(deviceIDs[deviceId], CL_DEVICE_NAME, 0, NULL, &paramValueSize);
            checkError(errNum, "Failed to find OpenCL device info");

            
            info = (Char *)alloca(sizeof(Char) * paramValueSize);  // String to display vendor name info
            errNum = clGetDeviceInfo(deviceIDs[deviceId], CL_DEVICE_NAME, paramValueSize, info, NULL);
            checkError(errNum, "Failed to find OpenCL device info" );
            
            // Create Context
            cl_context_properties  contextProperties[] = {CL_CONTEXT_PLATFORM, (cl_context_properties)platformIDs[i],0 };
            context = clCreateContext(contextProperties, numDevices, deviceIDs, &contextCallback, NULL, &errNum);
            deviceFound = checkError(errNum, "clCreateContext");         
            
        }   
    }
    if(deviceFound)
        printf("Using GPU device              : %s\n", info);
    return deviceFound;
}

Bool TEncOpenCL::compileKernelSource(const Char* fileName, const Char* kernelNameCalc )
{
    
    compileKernel = true;  
	    
    //read external kernel
    std::ifstream srcFile(fileName);    

    if(!checkError(srcFile.is_open() ? CL_SUCCESS : -1, "Reading Kernel\n"))
        return  false;
        
    
    //create string with external kernel
    std::string srcProg(std::istreambuf_iterator<char>(srcFile),(std::istreambuf_iterator<char>()));

    //pass string kernel to array char 
    kernelSource = srcProg.c_str();
    size_t length = srcProg.length();

    // Create program from source
    program = clCreateProgramWithSource(context, 1, &kernelSource, &length, &errNum);
    compileKernel = checkError(errNum, "clCreateProgramWithSource");   
    
      
    // Build program
    errNum = clBuildProgram(program, numDevices, deviceIDs, NULL, NULL, NULL);//error
    if (errNum != CL_SUCCESS)
    {
        // Determine the reason for the error
        Char buildLog[16384];
        clGetProgramBuildInfo( program, deviceIDs[deviceId], CL_PROGRAM_BUILD_LOG, 16384, buildLog,	NULL);
        
        fprintf(stderr, "Error in kernel: " );
        fprintf(stderr,"Compile Error: %s\n" , buildLog);
    }    
    
    
    //Create kernel object to calculate
    kernelCalc = clCreateKernel(program, kernelNameCalc, &errNum);
    compileKernel = checkError(errNum, "clCreateKernelCalc");
    
    //Create kernel object to compare
    kernelCompare = clCreateKernel(program, "compareSAD", &errNum);
    compileKernel = checkError(errNum, "clCreateKernelCompare");

    //Create a Command Queue to run kernels in the device
    queue = clCreateCommandQueue(context,deviceIDs[deviceId], 0, &errNum);
    compileKernel = checkError(errNum, "clCreateCommandQueue");
    
    return compileKernel;
       
}

/*  Create OpenCL buffers and initialize arrays

 */
Bool TEncOpenCL::createBuffers(UInt i_maxCtuWidth, UInt i_maxCtuHeight, Int i_searchRange)
{                
    Bool init = true;
    
    maxCtuWidth         = i_maxCtuWidth;
    maxCtuHeight        = i_maxCtuHeight;
    searchRange         = (i_searchRange << 1);
     
    Xarray              = (Int*         )xMalloc(Int, NUM_CTU_PARTS);        
    Yarray              = (Int*         )xMalloc(Int, NUM_CTU_PARTS);           
    minSad              = (Distortion*  )xMalloc(Distortion, NUM_CTU_PARTS);
    ruiCosts            = (Distortion*  )xMalloc(Distortion, NUM_CTU_PARTS);
    srchAreaPtr         = (Pel*         )xMalloc(Pel, (searchRange + maxCtuWidth) * (searchRange + maxCtuHeight));
    
    //Create Buffer of CTU Pixels
    pelCtuBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(Pel) * (maxCtuWidth * maxCtuHeight), NULL , &errNum);
    init = checkError(errNum, "clCreateBuffer(blockPixelBuffer)");              
    
    // Create Buffer of Search Area Pixels
    pelAreaBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(Pel) * (searchRange + maxCtuWidth) * (searchRange + maxCtuHeight), NULL, &errNum);
    init = checkError(errNum, "clCreateBuffer(areaPixelBuffer)"); 
    
    // Create Buffer of minimum SAD 
    sadBuffer = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(Distortion) * NUM_CTU_PARTS, NULL, &errNum);
    init = checkError(errNum, "clCreateBuffer(sad Buffer)"); 
    
    // Create Buffer of minimum SAD 
    ruiCostBuffer = clCreateBuffer(context, CL_MEM_WRITE_ONLY, sizeof(Distortion) * NUM_CTU_PARTS, NULL, &errNum);
    init = checkError(errNum, "clCreateBuffer(sad Buffer)"); 
    
    // Create Buffer of SAD calculated currently
    tempSadBuffer = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(Distortion) * NUM_CTU_PARTS, NULL, &errNum);
    init = checkError(errNum, "clCreateBuffer(temp SadBuffer)"); 
    
    // Create Buffer of position X of minimum SAD 
    XarrayBuffer = clCreateBuffer(context, CL_MEM_WRITE_ONLY, sizeof(Int) * NUM_CTU_PARTS, NULL, &errNum);
    init = checkError(errNum, "clCreateBuffer(XarrayBuffer)"); 
    
    // Create Buffer of position Y of minimum SAD 
    YarrayBuffer = clCreateBuffer(context, CL_MEM_WRITE_ONLY, sizeof(Int) * NUM_CTU_PARTS, NULL, &errNum);
    init = checkError(errNum, "clCreateBuffer(YarrayBuffer)");  
 
    return init;
}

Void TEncOpenCL::calcMotionVectors(Pel* pelCtu, Pel* pelSearch, Int i_iRefStride, Int i_iCtuStride, Int i_areaSize, TComMv* pcMvSrchRngLT)
{           
    
    Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
    Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
    
    
    iRefStride          = i_iRefStride; 
    iCtuStride          = i_iCtuStride;
    
    // get Initial position of Search array
    pelSearchArray      = pelSearch + (iRefStride * iSrchRngVerTop) + iSrchRngHorLeft ; 
    pelCtuArray         = pelCtu;
    areaSize            = (i_areaSize << 1);
   
    //printf("pel: %d\n", *pelSearchArray);
    Int     areaStride  = areaSize + iCtuStride;
    const size_t  globalIndex = iCtuStride >> 2;              
    
    // Initialize kernel work dimensions 
    const size_t globalWorkSizeCalc[2] = {globalIndex, globalIndex};  
    const size_t localWorkSizeCalc[2]  = {globalIndex, globalIndex};
    
    const size_t globalWorkSizeCompare[1] = {NUM_CTU_PARTS};
    const size_t localWorkSizeCompare[1]  = {1};

    // Transfer Ctu pixels to CtuBuffer
    errNum = clEnqueueWriteBuffer ( queue, pelCtuBuffer, CL_TRUE, 0, sizeof(Pel) * (iCtuStride * iCtuStride), (void *)pelCtu, 0, NULL, NULL);
    checkError(errNum, "clEnqueueMapBuffer - pelCtuBuffer");
    
    // Transfer Search Area pixels to Area Buffer
    srchAreaPtr = (Pel* )clEnqueueMapBuffer ( queue, pelAreaBuffer, CL_TRUE, CL_MAP_WRITE, 0, sizeof(Pel) * ((areaStride) * (areaStride)), 0, NULL, NULL, &errNum);
    checkError(errNum, "clEnqueueMapBuffer - pelAreaBuffer");
    
    
    for(Int i = 0; i< areaStride; i++)
        for(Int j= 0; j < areaStride; j++)
            srchAreaPtr[j + (i * areaStride)] = pelSearchArray[j + (i * iRefStride)];
    
    errNum = clEnqueueUnmapMemObject(queue, pelAreaBuffer, srchAreaPtr, 0, NULL, NULL);
    checkError(errNum, "clEnqueueUnmapMemObject - pelAreaBuffer");   
   
    // Reset Arrays values
    xResetArrays(); 
    xFillSADBuffer();
    
    //Set Arguments to Kernel that calculate recursive sad
//------------------------------------------------------------------------------------------------------------------------
    
    errNum  = clSetKernelArg(kernelCalc, 0, sizeof(cl_mem), &pelCtuBuffer);
    errNum |= clSetKernelArg(kernelCalc, 1, sizeof(cl_mem), &pelAreaBuffer);
    errNum |= clSetKernelArg(kernelCalc, 2, 256 * sizeof(Distortion), NULL);
    errNum |= clSetKernelArg(kernelCalc, 3, 256 * sizeof(Distortion), NULL);
    errNum |= clSetKernelArg(kernelCalc, 4, 256 * sizeof(Distortion), NULL);
    errNum |= clSetKernelArg(kernelCalc, 5, sizeof(cl_mem), &tempSadBuffer);
    errNum |= clSetKernelArg(kernelCalc, 6, sizeof(Int),    &areaStride);
    errNum |= clSetKernelArg(kernelCalc, 7, sizeof(Int),    &iCtuStride);
    
    checkError(errNum, "clSetKernelArg - kernelCalc");
    
     //Set Arguments to Kernel that compare sad
//------------------------------------------------------------------------------------------------------------------------ 

    errNum = clSetKernelArg(kernelCompare, 0, sizeof(cl_mem),  &tempSadBuffer);
    errNum |= clSetKernelArg(kernelCompare, 1, sizeof(cl_mem), &sadBuffer);
    errNum |= clSetKernelArg(kernelCompare, 2, sizeof(cl_mem), &XarrayBuffer);
    errNum |= clSetKernelArg(kernelCompare, 3, sizeof(cl_mem), &YarrayBuffer);
    errNum |= clSetKernelArg(kernelCompare, 4, sizeof(cl_mem), &ruiCostBuffer);
    errNum |= clSetKernelArg(kernelCompare, 5, sizeof(Int),    &m_lambda);
    
    checkError(errNum, "clSetKernelArg - kernelCompare"); 
//--------------------------------------------------------------------------------------------------------------------------
    for( Int y = 0; y <= areaSize; y++)
        for(Int x = 0; x <= areaSize; x++)
        {        
         
            errNum |= clSetKernelArg(kernelCalc, 8, sizeof(Int), &x);
            errNum |= clSetKernelArg(kernelCalc, 9, sizeof(Int), &y);
            
            // Execute kernel to calculate recursive sad
            errNum = clEnqueueNDRangeKernel(queue, kernelCalc, 2, NULL, globalWorkSizeCalc, localWorkSizeCalc, 0, NULL, NULL);
            checkError(errNum, "clEnqueueNDRangeKernel - Kernel Calc");           
                     
            Int posX = x + iSrchRngHorLeft;
            Int posY = y + iSrchRngVerTop;
            
            errNum |= clSetKernelArg(kernelCompare, 6, sizeof(Int), &posX);
            errNum |= clSetKernelArg(kernelCompare, 7, sizeof(Int), &posY);
            
            // Execute kernel to compare min sad
            errNum = clEnqueueNDRangeKernel(queue, kernelCompare, 1, NULL, globalWorkSizeCompare, localWorkSizeCompare, 0, NULL, NULL);
            checkError(errNum, "clEnqueueNDRangeKernel - Kernel Compare"); 
          
	}
    
    clFlush(queue);
    
    // Read Buffers and transfer data to CPU arrays
    minSad      = (Distortion* )clEnqueueMapBuffer(queue, sadBuffer, CL_TRUE, CL_MAP_READ, 0, sizeof(Distortion) * NUM_CTU_PARTS, 0, NULL, NULL, &errNum);
    checkError(errNum, "clEnqueueMapBuffer - sadBuffer");
    
    ruiCosts    = (Distortion* )clEnqueueMapBuffer(queue, ruiCostBuffer, CL_TRUE, CL_MAP_READ, 0, sizeof(Distortion) * NUM_CTU_PARTS, 0, NULL, NULL, &errNum);
    checkError(errNum, "clEnqueueMapBuffer - sadBuffer");
    
    Xarray      = (Int* )clEnqueueMapBuffer(queue, XarrayBuffer, CL_TRUE, CL_MAP_READ, 0, sizeof(Int) * NUM_CTU_PARTS, 0, NULL, NULL, &errNum);
    checkError(errNum, "clEnqueueMapBuffer - XarrayBuffer");
    
    Yarray      = (Int *)clEnqueueMapBuffer(queue, YarrayBuffer, CL_TRUE, CL_MAP_READ, 0, sizeof(Int) * NUM_CTU_PARTS, 0, NULL, NULL, &errNum);
    checkError(errNum, "clEnqueueMapBuffer - YarrayBuffer");    
   
    errNum = clEnqueueUnmapMemObject(queue, sadBuffer, minSad, 0, NULL, NULL);
    checkError(errNum, "clEnqueueUnmapMemObject - sadBuffer");
    
    errNum = clEnqueueUnmapMemObject(queue, ruiCostBuffer, ruiCosts, 0, NULL, NULL);
    checkError(errNum, "clEnqueueUnmapMemObject - ruiCostBuffer");
    
    errNum = clEnqueueUnmapMemObject(queue, XarrayBuffer, Xarray, 0, NULL, NULL);
    checkError(errNum, "clEnqueueUnmapMemObject - XarrayBuffer");
    
    errNum = clEnqueueUnmapMemObject(queue, YarrayBuffer, Yarray, 0, NULL, NULL);
    checkError(errNum, "clEnqueueUnmapMemObject - YarrayBuffer");
    
}
/*
 Function to fill Buffers with zeros and max values
 */
Void TEncOpenCL::xFillSADBuffer()
{
    Distortion max = std::numeric_limits<Distortion>::max();
    cl_int zero = 0;
    
    errNum = clEnqueueFillBuffer (queue, sadBuffer, (void *)(&max), sizeof(Distortion), 0, sizeof(Distortion) * NUM_CTU_PARTS, 0, NULL, NULL );
    checkError(errNum, "clEnqueueFillBuffer - sad Buffer");
    
    errNum = clEnqueueFillBuffer (queue, tempSadBuffer, (void *)(&zero), sizeof(Distortion), 0, sizeof(Distortion) * 256, 0, NULL, NULL );
    checkError(errNum, "clEnqueueFillBuffer - tempSad Buffer");
    
    errNum = clEnqueueFillBuffer (queue, XarrayBuffer,  (void *)(&zero), sizeof(Int), 0,sizeof(Int) * NUM_CTU_PARTS, 0, NULL, NULL );
    checkError(errNum, "clEnqueueFillBuffer - XarrayBuffer");
    
    errNum = clEnqueueFillBuffer (queue, YarrayBuffer, (void *)(&zero), sizeof(Int), 0, sizeof(Int) * NUM_CTU_PARTS, 0, NULL, NULL );
    checkError(errNum, "clEnqueueFillBuffer - YarrayBuffer");
    
}

Void TEncOpenCL::xResetArrays()
{
    for(int i = 0; i< NUM_CTU_PARTS; i++)
    {
        Xarray[i] = 0;
        Yarray[i] = 0;
    }
}
