#include "mex.h"
#include "tracklets.h"

using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  
  // buffers and file name
  char   *input_buf;
  char   *output_buf;
  string  filename;

  // check for proper number of arguments
  if (nrhs!=1) 
    mexErrMsgTxt("One input required (filename).");
  if (nlhs!=1) 
    mexErrMsgTxt("One output required (tracklets).");

  // check arguments
  if (mxIsChar(prhs[0])!=1 || mxGetM(prhs[0])!=1)
    mexErrMsgTxt("Filename must be a string.");

  // copy the string data from prhs[0] into a C string input_ buf.
  input_buf = mxArrayToString(prhs[0]);

  // read tracklets
  filename = input_buf;
  Tracklets *tracklets = new Tracklets();
  if (!tracklets->loadFromFile(filename))
    return;
  
  // create output cell array
  int      ndim = 1, dims[]={tracklets->numberOfTracklets(), 1}; 
  mxArray  *cell_array_ptr;
  cell_array_ptr = mxCreateCellArray(ndim, dims);
  
  // create struct for tracklet
  int              s_ndim = 2, s_dims[2] = {1,1};
  int              number_of_fields = 7; 
  const char       *field_names[] = {"objectType", "h", "w", "l", "first_frame", "poses", "finished"};
  Tracklets::tPose *pose;
  double           *s_pr;
  mxArray          *field_value, *struct_array_ptr;

  // read tracklets
  for (int tracklet_idx=0; tracklet_idx<tracklets->numberOfTracklets(); tracklet_idx++) {
    
    // set the struct size for the current tracklet
    struct_array_ptr = mxCreateStructArray(s_ndim, s_dims, number_of_fields, field_names);
    
    // populate the name fields
    field_value = mxCreateString(tracklets->getTracklet(tracklet_idx)->objectType.c_str());
    mxSetField(struct_array_ptr, 0, "objectType", field_value);
    field_value = mxCreateDoubleMatrix(1, 1, mxREAL);
    s_pr        = mxGetPr(field_value);
    s_pr[0]     = tracklets->getTracklet(tracklet_idx)->h;
    mxSetField(struct_array_ptr, 0, "h", field_value); 
    field_value = mxCreateDoubleMatrix(1, 1, mxREAL);
    s_pr        = mxGetPr(field_value);
    s_pr[0]     = tracklets->getTracklet(tracklet_idx)->w;
    mxSetField(struct_array_ptr, 0, "w", field_value); 
    field_value = mxCreateDoubleMatrix(1, 1, mxREAL);
    s_pr        = mxGetPr(field_value);
    s_pr[0]     = tracklets->getTracklet(tracklet_idx)->l;
    mxSetField(struct_array_ptr, 0, "l", field_value); 
    field_value = mxCreateDoubleMatrix(1, 1, mxREAL);
    s_pr        = mxGetPr(field_value);
    s_pr[0]     = tracklets->getTracklet(tracklet_idx)->first_frame;
    mxSetField(struct_array_ptr, 0, "first_frame", field_value); 
    field_value = mxCreateDoubleMatrix(1, 1, mxREAL);
    s_pr        = mxGetPr(field_value);
    s_pr[0]     = tracklets->getTracklet(tracklet_idx)->finished;
    mxSetField(struct_array_ptr, 0, "finished", field_value); 

    // set size of poses
    const int poses_dim[] = {15, tracklets->getTracklet(tracklet_idx)->poses.size()};
    field_value = mxCreateNumericArray(2, poses_dim,mxDOUBLE_CLASS,mxREAL);
    s_pr        = mxGetPr(field_value);
    
    // read poses
    int k=0;
    for (int pose_idx=0; pose_idx<tracklets->getTracklet(tracklet_idx)->poses.size(); pose_idx++) {
      tracklets->getPose(tracklet_idx, pose_idx+tracklets->getTracklet(tracklet_idx)->first_frame, pose);
      *(s_pr+k++) = pose->tx;
      *(s_pr+k++) = pose->ty;
      *(s_pr+k++) = pose->tz;
      *(s_pr+k++) = pose->rx;
      *(s_pr+k++) = pose->ry;
      *(s_pr+k++) = pose->rz;
      *(s_pr+k++) = pose->state;
      *(s_pr+k++) = pose->occlusion;
      *(s_pr+k++) = pose->occlusion_kf;
      *(s_pr+k++) = pose->truncation;
      *(s_pr+k++) = pose->amt_occlusion;
      *(s_pr+k++) = pose->amt_occlusion_kf;
      *(s_pr+k++) = pose->amt_border_l;
      *(s_pr+k++) = pose->amt_border_r;
      *(s_pr+k++) = pose->amt_border_kf;
    }
    mxSetField(struct_array_ptr, 0, "poses", field_value);
    mxSetCell(cell_array_ptr, tracklet_idx, struct_array_ptr);
  }
  
  // set output data to pointer
  plhs[0] = cell_array_ptr;
  
  // output
  cout << "Found " << tracklets->numberOfTracklets() << " tracklets." << endl;
  
  // clean up
  mxFree(input_buf);
  delete tracklets;
  return;
}
