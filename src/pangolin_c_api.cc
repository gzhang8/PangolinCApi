//
// Created by gzhang8 on 5/5/20.
//

#include <string>
// #include <Eigen/Core>
#include <Eigen/Geometry>
#include <pangolin/pangolin.h>
#include <pangolin/display/display_internal.h>

extern "C"
{


void pangolin_create_window(char* window_name, int width, int height, int panel) {

    if (pangolin::FindContext(std::string(window_name)) != NULL){
        return;
    }

    width += panel;

    pangolin::Params windowParams;

    windowParams.Set("SAMPLE_BUFFERS", 0);
    windowParams.Set("SAMPLES", 0);

    pangolin::CreateWindowAndBind(std::string(window_name), width, height, windowParams);

}

void* pangolin_create_render_state(
        double* proj_mat_ptr,
        double* model_view_ptr
        ){
    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState* s_cam_ptr =  new pangolin::OpenGlRenderState(
        pangolin::OpenGlMatrix::ColMajor4x4(proj_mat_ptr),
        pangolin::OpenGlMatrix::ColMajor4x4(model_view_ptr)
    );

    return reinterpret_cast<void*>(s_cam_ptr);
}

void* pangolin_create_render_state_no_params(){
    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState* s_cam_ptr =  new pangolin::OpenGlRenderState(
             pangolin::ProjectionMatrix(640,480,525,525,320,240,0.1,1000),
             pangolin::ModelViewLookAt(0, 0 , -1, 0, 0, 1, pangolin::AxisNegY)
    );

    return reinterpret_cast<void*>(s_cam_ptr);
}

void pangolin_get_projection_model_view_matrix(void* s_cam_void_ptr, double* out){
    pangolin::OpenGlRenderState* s_cam_ptr = (pangolin::OpenGlRenderState*)s_cam_void_ptr;
    memcpy(out, s_cam_ptr->GetProjectionModelViewMatrix().m, 16*sizeof(double));

}

void pangolin_set_model_view_matrix(void* s_cam_void_ptr, double* in){
    pangolin::OpenGlRenderState* s_cam_ptr = (pangolin::OpenGlRenderState*)s_cam_void_ptr;
    s_cam_ptr->SetModelViewMatrix(pangolin::OpenGlMatrix::ColMajor4x4(in));
}

void pangolin_set_model_view_matrix_with_conversion(
        void* s_cam_void_ptr, double* in) {
    pangolin::OpenGlMatrix mv;

    Eigen::Map<Eigen::Matrix<double,4,4,Eigen::ColMajor> > currPose(in);
    Eigen::Matrix3d currRot = currPose.topLeftCorner(3, 3);

    Eigen::Quaterniond currQuat(currRot);
    Eigen::Vector3d forwardVector(0, 0, 1);
    bool iclnuim = false;
    Eigen::Vector3d upVector(0, iclnuim ? 1 : -1, 0);

    Eigen::Vector3d forward = (currQuat * forwardVector).normalized();
    Eigen::Vector3d up = (currQuat * upVector).normalized();

    Eigen::Vector3d eye(currPose(0, 3), currPose(1, 3), currPose(2, 3));

    eye -= forward;

    Eigen::Vector3d at = eye + forward;

    Eigen::Vector3d z = (eye - at).normalized();  // Forward
    Eigen::Vector3d x = up.cross(z).normalized(); // Right
    Eigen::Vector3d y = z.cross(x);

    Eigen::Matrix4d m;// = fix_view_pose.cast<double>();
    m << x(0),  x(1),  x(2),  -(x.dot(eye)),
        y(0),  y(1),  y(2),  -(y.dot(eye)),
        z(0),  z(1),  z(2),  -(z.dot(eye)),
        0,     0,     0,              1;

    memcpy(&mv.m[0], m.data(), sizeof(Eigen::Matrix4d));

    pangolin::OpenGlRenderState* s_cam_ptr = (pangolin::OpenGlRenderState*)s_cam_void_ptr;
    s_cam_ptr->SetModelViewMatrix(mv);
}


void* pangolin_create_handler3d(void* renderstate_ptr){
    pangolin::OpenGlRenderState* s_cam_ptr = reinterpret_cast<pangolin::OpenGlRenderState*>(renderstate_ptr);
    pangolin::Handler3D* handler_ptr = new pangolin::Handler3D(*s_cam_ptr); //(s_cam);
    return (void*) handler_ptr;
}

// return a view in window
void* pangolin_create_display(void* handler_void_ptr){

    // Create Interactive View in window
    pangolin::Handler3D* handler_ptr = (pangolin::Handler3D*)handler_void_ptr;
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(handler_ptr);

    return &d_cam;
}

// 1 means true; 0 -> false
int pangolin_should_quit(){
    int res = pangolin::ShouldQuit()? 1:0;
    // reset flag
    pangolin::GetCurrentContext()->quit = false;
    return res;
}

// first is a view: d_cam
// second is a rendersate: s_cam
void pangolin_view_active_render_sate(void* view_void_ptr, void* rendersate_ptr){
    pangolin::OpenGlRenderState* s_cam_ptr = reinterpret_cast<pangolin::OpenGlRenderState*>(rendersate_ptr);
    pangolin::View* d_cam_ptr = reinterpret_cast<pangolin::View*>(view_void_ptr);
    d_cam_ptr->Activate(*s_cam_ptr);

}

void pangolin_view_show(void* view_void_ptr, int flag){
    pangolin::View* d_cam_ptr = reinterpret_cast<pangolin::View*>(view_void_ptr);
    d_cam_ptr->Show(flag != 0);
}

// Swap frames and Process Events
void pangolin_finish_frame(){
    pangolin::FinishFrame();
}

void pangolin_destroy_window(char* window_name){
    pangolin::DestroyWindow(std::string(window_name));
}

// for testing
void pangolin_glDrawColouredCube(){
    pangolin::glDrawColouredCube();

}

int make_window_context_current(char* window_name){
    pangolin::PangolinGl * ctx = pangolin::FindContext(std::string(window_name));
    if (ctx != NULL){
        ctx->MakeCurrent();
        return 1;
    } else {
        printf("Warning: cannot find the window name");
        return 0;
    }
}

} // end extern C
