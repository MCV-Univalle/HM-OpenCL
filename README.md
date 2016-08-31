# HM-OpenCL

<b>HEVC Test Model With OpenCL Motion Estimation</b><br>

HEVC Test Model version 16.4 with a module for performing the Motion Estimation process using a GPU. In this proyect is used OpenCL as parallel programming framework.

<h2>Added features</h2>
<ul>
  <li><b>TEncOpenCL class:</b> Module for Motion Estimation with OpenCL and it was included in source/Lib/TLibEncoder/ path. This class performs two task:</li>
    <ul>
      <li>Check the OpenCL conditions for the system (search a OpenCL device support, init OpenCL variables)</li>
      <li>Execute the module in the GPU</li>
    </ul>
  <li><b>TEncOpenCL::findDevices method:</b>verify that the system meets the conditions for running OpenCL (search OpenCL platform and search OpenCL device)</li>
  <li><b>TEncOpenCL::compileKernelSource method:</b> initialize the OpenCL variables. Moreover read and compile the OpenCl kernel</li>
  <li><b>TEncOpenCL::calcMotionVectors method:</b> Load the pixels of CTU and search area into the input buffers,  calculate the sum of absolute differences for one CTU and store the information into temporal buffers for the calculus of motion vectors and rate-distortion cost</li>
</ul>

<h2>Modifications</h2>
<ul>
  <li><b>EncTop class:</b> function for calling the OpenCL methods was added. Based of system verification show a activation of OpenCL message </li>
  <li><b>TAppEncCfg class:</b> Options OpenCL, OpenCLDevice and  KernelOpenCL was added in the class in order to configuration file recognize them</li>
  <li><b>TEncSearch class:</b> two 3d-array was added:</li>
    <ul>
      <li>TComMv allMotionVectors[2][33][NUM_CTU_PARTS]</li>
      <li> Distortion allRuiCost[2][33][NUM_CTU_PARTS]</li>
    </ul>
</ul>
<h2>Configuration</h2>
For motion estimation with OpenCL is necessary to add to configuration file the following parameters:  <br>
<ul>
  <li>OpenCL:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; int &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;(0: disabled - 1: enabled) </li>
  <li>OpenCLDevice: &nbsp;&nbsp;int	&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;(OpenCL device ID) </li>
  <li>KernelOpenCL: :&nbsp;&nbsp;string &nbsp;&nbsp;&nbsp;(path of file "sad.cl") </li>
</ul>

For more information of HEVC Test Model can to visit the next link:<br>
<a>https://hevc.hhi.fraunhofer.de/</a>


