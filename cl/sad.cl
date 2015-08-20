#define BLOCK_WIDTH 4
#define BLOCK_HEIGHT 4

__kernel void calcSAD(  __global short*  block_pixel, __global short* area_pixel,
                        __local unsigned int* sadHorizontal, __local unsigned int* sadVertical, __local unsigned int* sadAMP,
                        __global unsigned int* tempSad, int iStrideCur, int iStrideOrg, int posX, int posY)
{
 // get indices relative to the work-item (16x16 matrix) 
    unsigned int i  = get_local_id(0);
    unsigned int j  = get_local_id(1); 
    
    // get size of each row of work items 
    unsigned int group = get_local_size(0);

    // index to convert pos (x,y) of each work-item in pos (i) 
    int index = i + (group * j);

    // index to put index in each 4x4 block in CTU block and Search Area 
    int xBlock   = i * BLOCK_WIDTH;
    int yBlock   = j * (iStrideOrg * BLOCK_HEIGHT);

    int blockInit = xBlock + yBlock;

     // index to put index in each 4x4 block in search area 
    int xArea    = posX + (i * BLOCK_WIDTH);
    int yArea    = (posY * iStrideCur) + (j * (iStrideCur * BLOCK_HEIGHT));
    
    int areaInit = xArea + yArea;
    
    unsigned int sum = 0;
    
    // calculate the sad for each 4x4 block (unrolled)              
    sum += abs_diff(block_pixel[blockInit],				area_pixel[areaInit]); 
    sum += abs_diff(block_pixel[blockInit + 1], 			area_pixel[areaInit + 1]);
    sum += abs_diff(block_pixel[blockInit + 2], 			area_pixel[areaInit + 2]);
    sum += abs_diff(block_pixel[blockInit + 3], 			area_pixel[areaInit + 3]);
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg)], 		area_pixel[areaInit + (iStrideCur)]); 
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg) + 1],		area_pixel[areaInit + (iStrideCur) + 1]);
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg) + 2],		area_pixel[areaInit + (iStrideCur) + 2]);
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg) + 3],		area_pixel[areaInit + (iStrideCur) + 3]);
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg * 2)], 		area_pixel[areaInit + (iStrideCur * 2)]); 
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg * 2) + 1], 	area_pixel[areaInit + (iStrideCur * 2) + 1]);
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg * 2) + 2], 	area_pixel[areaInit + (iStrideCur * 2) + 2]);
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg * 2) + 3], 	area_pixel[areaInit + (iStrideCur * 2) + 3]);
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg * 3)],		area_pixel[areaInit + (iStrideCur * 3)]); 
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg * 3) + 1], 	area_pixel[areaInit + (iStrideCur * 3) + 1]);
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg * 3) + 2], 	area_pixel[areaInit + (iStrideCur * 3) + 2]);
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg * 3) + 3], 	area_pixel[areaInit + (iStrideCur * 3) + 3]);
    

    barrier(CLK_GLOBAL_MEM_FENCE);
  
     /* allocate the result the each 4x4 sad in local memory for calculate vertical recursive sad*/
    sadVertical[index] = sum;
    barrier(CLK_LOCAL_MEM_FENCE);

    /*create index to move in horizontal and vertical positions*/
    uint indexH = index * 2;
    uint indexV = (j * 32 + i);
    uint idxV = (j*4) + i;
    
    /* calculate sad of 128 blocks of 8x4 and transfer to 8x4 buffer*/
    tempSad[index] = sadVertical[indexH] + sadVertical[indexH + 1];
    barrier(CLK_LOCAL_MEM_FENCE);
        
    /* calculate sad of 128 blocks of 4x8 and transfer to 4x8 buffer*/
    sadVertical[index] = sadVertical[indexV] + sadVertical[indexV+16];
    barrier(CLK_LOCAL_MEM_FENCE);
    
    tempSad[index + 128] = sadVertical[index];
    barrier(CLK_LOCAL_MEM_FENCE);

    /* calculate sad of 64 blocks of 8x8 and transfer to 8x8 buffer */
    sadVertical[index] = sadVertical[indexH] + sadVertical[indexH + 1];
    barrier(CLK_LOCAL_MEM_FENCE);

    tempSad[index + 256] = sadVertical[index];
    barrier(CLK_LOCAL_MEM_FENCE);

    /* calculate sad of 32 blocks of 16x8 and transfer to 16x8 buffer)*/ 
    tempSad[index + 320] = sadVertical[indexH] + sadVertical[indexH + 1];
    barrier(CLK_LOCAL_MEM_FENCE);
    
     /* calculate sad of 32 blocks of 8x16 and transfer to 8x16 buffer*/
    indexV = (j * 16 + i);
    idxV = (j * 8) + i;
    if(i<8)
    {
        sadVertical[idxV] = sadVertical[indexV] + sadVertical[indexV + 8];
        barrier(CLK_LOCAL_MEM_FENCE);
    }
     
    tempSad[index + 352] = sadVertical[index];

    /* calculate sad of 16 blocks of 16x16 (first allocate in temp sadVertical, and later transfer to 16x16 buffer)*/
    sadVertical[index] = sadVertical[indexH] + sadVertical[indexH + 1];
    barrier(CLK_LOCAL_MEM_FENCE);

    tempSad[index + 384] = sadVertical[index];
    barrier(CLK_LOCAL_MEM_FENCE);   

    /* calculate sad of 8 blocks of 32x16 (transfer directly to 32x16 buffer)*/
    sadVertical[index] = sadVertical[indexH] + sadVertical[indexH + 1];
    barrier(CLK_LOCAL_MEM_FENCE);
    
    tempSad[index + 400] = sadHorizontal[index];
    
      /* calculate sad of 8 blocks of 16x32 (first allocate in temp sadVertical, and later transfer to 16x32 buffer)*/
    indexV = (j * 8 + i);
    if(i<4)
    {
        sadVertical[(j*4) + i] = sadVertical[indexV] + sadVertical[indexV + 4];
        barrier(CLK_LOCAL_MEM_FENCE);
    }
    
    tempSad[index + 408] = sadVertical[index];

    /* calculate sad of 4 blocks of 32x32 (first allocate in temp sadVertical, and later transfer to 32x32 buffer)*/
    sadVertical[index] = sadVertical[indexH] + sadVertical[indexH + 1];
    barrier(CLK_LOCAL_MEM_FENCE);

    tempSad[index + 416] = sadVertical[index];
    
     /* calculate sad of 2 blocks of 64x32 (transfer directly to 64x32 buffer)*/
    tempSad[index + 420] = sadVertical[indexH] + sadVertical[indexH + 1];
    barrier(CLK_LOCAL_MEM_FENCE);  
    
    /* calculate sad of 2 blocks of 32x64 (first allocate in temp sadVertical, and later transfer to 32x64 buffer)*/
    sadVertical[index] = sadVertical[index] + sadVertical[index + 2];
    barrier(CLK_LOCAL_MEM_FENCE);    
    
    tempSad[index + 422] = sadVertical[index];  
    
    /* calculate sad of 64x64 block and transfer directly to 64x32 buffer)*/
    tempSad[index + 424] = sadVertical[index] + sadVertical[index + 1];
    barrier(CLK_GLOBAL_MEM_FENCE); 
    
}


__kernel void calcSAD_AMP(  __global short*  block_pixel, __global short* area_pixel,
                        __local unsigned int* sadHorizontal, __local unsigned int* sadVertical, __local unsigned int* sadAMP,
                        __global unsigned int* tempSad, int iStrideCur, int iStrideOrg, int posX, int posY)

{   
    // get indices relative to the work-item (16x16 matrix) 
    unsigned int i  = get_local_id(0);
    unsigned int j  = get_local_id(1); 
    
    // get size of each row of work items 
    unsigned int group = get_local_size(0);

    // index to convert pos (x,y) of each work-item in pos (i) 
    int index = i + (group * j);

    // index to put index in each 4x4 block in CTU block and Search Area 
    int xBlock   = i * BLOCK_WIDTH;
    int yBlock   = j * (iStrideOrg * BLOCK_HEIGHT);

    int blockInit = xBlock + yBlock;

     // index to put index in each 4x4 block in search area 
    int xArea    = posX + (i * BLOCK_WIDTH);
    int yArea    = (posY * iStrideCur) + (j * (iStrideCur * BLOCK_HEIGHT));
    
    int areaInit = xArea + yArea;
    
    unsigned int sum = 0;
    
    // calculate the sad for each 4x4 block (unrolled)              
    sum += abs_diff(block_pixel[blockInit],				area_pixel[areaInit]); 
    sum += abs_diff(block_pixel[blockInit + 1], 			area_pixel[areaInit + 1]);
    sum += abs_diff(block_pixel[blockInit + 2], 			area_pixel[areaInit + 2]);
    sum += abs_diff(block_pixel[blockInit + 3], 			area_pixel[areaInit + 3]);
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg)], 		area_pixel[areaInit + (iStrideCur)]); 
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg) + 1],		area_pixel[areaInit + (iStrideCur) + 1]);
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg) + 2],		area_pixel[areaInit + (iStrideCur) + 2]);
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg) + 3],		area_pixel[areaInit + (iStrideCur) + 3]);
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg * 2)], 		area_pixel[areaInit + (iStrideCur * 2)]); 
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg * 2) + 1], 	area_pixel[areaInit + (iStrideCur * 2) + 1]);
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg * 2) + 2], 	area_pixel[areaInit + (iStrideCur * 2) + 2]);
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg * 2) + 3], 	area_pixel[areaInit + (iStrideCur * 2) + 3]);
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg * 3)],		area_pixel[areaInit + (iStrideCur * 3)]); 
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg * 3) + 1], 	area_pixel[areaInit + (iStrideCur * 3) + 1]);
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg * 3) + 2], 	area_pixel[areaInit + (iStrideCur * 3) + 2]);
    sum += abs_diff(block_pixel[blockInit + (iStrideOrg * 3) + 3], 	area_pixel[areaInit + (iStrideCur * 3) + 3]);
  
     /* allocate the result the each 4x4 sad in local memory for calculate vertical recursive sad*/
    sadVertical[index] = sum;
    barrier(CLK_LOCAL_MEM_FENCE);
        
    /*create index to move in horizontal and vertical positions*/
    int indexH = index * 2;
    int indexV = (j * 32 + i);
    int idxV = (j*4) + i;
    
    /* calculate sad of 128 blocks of 8x4 and transfer to 8x4 buffer*/
    sadHorizontal[index] = sadVertical[indexH] + sadVertical[indexH + 1];
    
    tempSad[index] = sadHorizontal[index];
    
    /* calculate sad of 128 blocks of 4x8 and transfer to 4x8 buffer*/
    sadVertical[index] = sadVertical[indexV] + sadVertical[indexV+16];
    
    tempSad[index + 128] = sadVertical[index];  
    
    /*calculate all 16x4 blocks*/
    sadAMP[index] = sadHorizontal[indexH] + sadHorizontal[indexH + 1];
    
    /*calculate 16x4(Up), 16x4(Down), 16x12(Up) and 16x12(Down) blocks*/
    indexV = (j * 16 + i);
    
    if(i<4)
    {
        //16x4(Up)
        tempSad[idxV + 256] = sadAMP[indexV];
        barrier(CLK_LOCAL_MEM_FENCE);
        //16x4(Down)
        tempSad[idxV + 272] = sadAMP[indexV + 12];
         barrier(CLK_LOCAL_MEM_FENCE);
        //16x12(Up)
        tempSad[idxV + 288] = sadAMP[indexV] + sadAMP[indexV + 4] + sadAMP[indexV + 8];
        //16x12(Down)
        tempSad[idxV + 304] = sadAMP[indexV + 4] + sadAMP[indexV + 8] + sadAMP[indexV +12];

    }
   

    /*calculate all 4x16 blocks*/
    indexV = (j * 32) + i;
    sadAMP[index] = sadVertical[indexV] + sadVertical[indexV + 16];  
    
    /*calculate 4x16(Left), 4x16Down), 16x12(Up) and 16x12(Down) blocks*/
    
    if(i<4)
    {
        //4x16(Left)
        tempSad[idxV + 320] = sadAMP[idxV * 4];
        //4x16(Rigth)
        tempSad[idxV + 336] = sadAMP[(idxV * 4) + 3];
        //12x16(Left)
        tempSad[idxV + 352] = sadAMP[(idxV * 4)] + sadAMP[(idxV * 4) + 1] + sadAMP[(idxV * 4) + 2];
        //12x16(Rigth)
        tempSad[idxV + 368] = sadAMP[(idxV *4) + 1] + sadAMP[(idxV * 4) + 2] + sadAMP[(idxV * 4) + 3];        
    }
    
    /* calculate sad of 64 blocks of 8x8 and transfer to 8x8 buffer */
    sadVertical[index] = sadVertical[indexH] + sadVertical[indexH + 1];

    tempSad[index + 384] = sadVertical[index];
    
    
    /* calculate sad of 32 blocks of 16x8 and transfer to 16x8 buffer)*/ 
    sadHorizontal[index] = sadVertical[indexH] + sadVertical[indexH + 1];
       
    tempSad[index + 448] = sadHorizontal[index];
    
     /* calculate sad of 32 blocks of 8x16 and transfer to 8x16 buffer*/
    indexV = (j * 16 + i);
    idxV = (j * 8) + i;
    if(i<8)
        sadVertical[idxV] = sadVertical[indexV] + sadVertical[indexV + 8];
    
     
    tempSad[index + 480] = sadVertical[index];
    
    /* calculate all 32x8 blocks */
    sadAMP[index] = sadHorizontal[indexH] + sadHorizontal[indexH + 1];
    
    idxV = (j * 2) + i;
    indexV = (j * 8) + i;
    
    /* calculate 32x8(Up), 32x8(Down), 32x24(Up) and 32x24(Down) */
    if(i<2)
    {
        //sad32x8(Up)
        tempSad[idxV + 512] = sadAMP[indexV];
        //32x8(Down)
        tempSad[idxV + 516] = sadAMP[indexV + 6];        
        //32x24(Up) 
        tempSad[idxV + 520] = sadAMP[indexV] + sadAMP[indexV + 2] + sadAMP[indexV + 4]; 
        //32x24(Down)
        tempSad[idxV + 524] = sadAMP[indexV + 2] + sadAMP[indexV + 4] + sadAMP[indexV + 6]; 
    }    
    
    /* calculate all 8x32 blocks */
    indexV = (j * 16) + i;
    idxV = (j * 8) + i;
    if(i<8)
          sadAMP[idxV] = sadVertical[indexV] + sadVertical[indexV + 8];
  
    /* calculate 8x32(Left), 8x32(Rigth), 24x32(Left) and 24x32(Rigth) sad */
    idxV = (j * 2) + i;
    if(i<2)
    {
        //8x32(Left)
         tempSad[idxV + 528] = sadAMP[idxV * 4];
        //8x32(Rigth)
         tempSad[idxV + 532] = sadAMP[(idxV * 4) + 3];
        //24x32(Left) 
        tempSad[idxV + 536] = sadAMP[idxV * 4] + sadAMP[(idxV * 4) + 1] + sadAMP[(idxV * 4) + 2];
        //24x32(Rigth)
        tempSad[idxV + 540] = sadAMP[(idxV * 4) + 1] + sadAMP[(idxV * 4) + 2] + sadAMP[(idxV * 4) + 3];
    }
    
     /* calculate sad of 16 blocks of 16x16 (first allocate in temp sadVertical, and later transfer to 16x16 buffer)*/
    sadVertical[index] = sadVertical[indexH] + sadVertical[indexH + 1];

    tempSad[index + 544] = sadVertical[index];      
    
    
    /* calculate sad of 8 blocks of 32x16 (transfer directly to 32x16 buffer)*/
    sadHorizontal[index] = sadVertical[indexH] + sadVertical[indexH + 1];
    
    tempSad[index + 560] = sadHorizontal[index];
    
      /* calculate sad of 8 blocks of 16x32 (first allocate in temp sadVertical, and later transfer to 16x32 buffer)*/
    indexV = (j * 8 + i);
    if(i<4)
        sadVertical[(j*4) + i] = sadVertical[indexV] + sadVertical[indexV + 4];
        
    
    
    tempSad[index + 568] = sadVertical[index];
    
    /*calculate all 64x16 blocks*/
    sadAMP[index] = sadHorizontal[indexH] + sadHorizontal[indexH + 1];
    
    if(i==0 && j==0)        
        tempSad[576] = sadAMP[0]; //sad64x16(Up)
    if(i==1 && j==0)    
        tempSad[577] = sadAMP[3]; //sad64x16(Down)
    if(i==2 && j==0)   
        tempSad[578] = sadAMP[0] + sadAMP[1] + sadAMP[2]; //sad64x48(Up)
    if(i==3 && j==0)    
        tempSad[579] = sadAMP[1] + sadAMP[2] + sadAMP[3]; //sad64x16(Down)
    
    sadAMP[index] =  sadVertical[index] + sadVertical[index + 4];
    
     if(i==0 && j==0)   
        tempSad[580] = sadAMP[0]; //sad64x16(Up)
     if(i==1 && j==0) 
        tempSad[581] = sadAMP[3]; //sad64x16(Down)
     if(i==2 && j==0)
        tempSad[582] = sadAMP[0] + sadAMP[1] + sadAMP[2]; //sad64x48(Up)
     if(i==3 && j==0)   
        tempSad[583] = sadAMP[1] + sadAMP[2] + sadAMP[3]; //sad64x16(Down)
    
    /* calculate sad of 4 blocks of 32x32 (first allocate in temp sadVertical, and later transfer to 32x32 buffer)*/
    sadVertical[index] = sadVertical[indexH] + sadVertical[indexH + 1];

    tempSad[index + 584] = sadVertical[index];
    
     /* calculate sad of 2 blocks of 64x32 (transfer directly to 64x32 buffer)*/
    sadHorizontal[index] = sadVertical[indexH] + sadVertical[indexH + 1];
    
    tempSad[index + 588] = sadHorizontal[index];
    
    /* calculate sad of 2 blocks of 32x64 (first allocate in temp sadVertical, and later transfer to 32x64 buffer)*/
    sadVertical[index] = sadVertical[index] + sadVertical[index + 2];  
    
    tempSad[index + 590] = sadVertical[index];  
    
    /* calculate sad of 64x64 block and transfer directly to 64x32 buffer)*/
    tempSad[index + 592] = sadVertical[index] + sadVertical[index + 1];
    barrier(CLK_LOCAL_MEM_FENCE);
}


__kernel void compareSAD(__global unsigned int* tempSad, __global unsigned int* minSad, __global  int* Xarray, __global  int* Yarray, __global unsigned int* ruiCost, int uiCost, int posX, int posY)

{
    unsigned int globalId = get_global_id(0);
    int x = posX * 4;
    int y = posY * 4;

    unsigned int uiLength = 1;
    unsigned int uiTemp   = ( x <= 0) ? (-x * 2 )+1: (x * 2);

    while ( 1 != uiTemp )
    {
       uiTemp = uiTemp / 2;
       uiLength += 2;
    }   
    
    unsigned int componentBitsX =  uiLength;
    
    uiLength = 1;
    uiTemp   = ( y <= 0) ? (-y * 2)+1: (y * 2);
    while ( 1 != uiTemp )
    {
       uiTemp = uiTemp / 2;
       uiLength += 2;
    } 

    unsigned int componentBitsY = uiLength;
 
    unsigned int sad = tempSad[globalId] + uiCost * (componentBitsX + componentBitsY) / 65536;
    
    if(sad < minSad[globalId])
    {
        minSad[globalId]  = sad;
	ruiCost[globalId] = tempSad[globalId];
        Xarray[globalId]  = posX;
        Yarray[globalId]  = posY;        
    }
    barrier(CLK_GLOBAL_MEM_FENCE);
}