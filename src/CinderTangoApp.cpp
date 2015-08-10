#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include <list>
#include "cinder/android/app/CinderNativeActivity.h"

#include "cinder/ImageIo.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Batch.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Shader.h"
#include "tango_client_api.h"
#include "cinder/android/JniHelper.h"
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include "CinderTango.h"

#include "tango-gl/conversions.h"
#include "tango-gl/util.h"
#include "cinder/Log.h"

using namespace ci;
using namespace ci::app;
using namespace std;

// We'll create a new Cinder Application by deriving from the App class
class CinderTangoApp : public App {
  public:
	void setup();
	void resize();
	void mouseDrag( MouseEvent event );
#if defined( CINDER_ANDROID )
    void touchesBegan( TouchEvent event );
    void touchesMoved( TouchEvent event );
#endif
	void keyDown( KeyEvent event );
	void update();
	void draw();
	void SetupExtrinsics();
	void SetupIntrinsics();

	gl::TextureCubeMapRef	mCubeMap;
	gl::BatchRef			mTeapotBatch, mGround;
	
	mat4				mObjectRotation;
	CameraPersp				mCam;
	bool tangoConnected;
	gl::TextureRef 	mPassThru;

	// This will maintain a list of points which we will draw line segments between
	list<vec2>		mPoints;

	// AR cube position in world coordinate.
	const glm::vec3 kCubePosition = glm::vec3(-1.0f, 0.265f, -2.0f);

	// AR grid position, can be modified based on the real world scene.
	const glm::vec3 kGridPosition = glm::vec3(0.0f, 1.26f, -2.0f);

	// AR cube dimension, based on real world scene.
	const glm::vec3 kCubeScale = glm::vec3(0.38f, 0.53f, 0.57f);

	// Marker scale.
	const glm::vec3 kMarkerScale = glm::vec3(0.05f, 0.05f, 0.05f);

	// Render camera observation distance in third person camera mode.
	const float kThirdPersonCameraDist = 7.0f;

	// Render camera observation distance in top down camera mode.
	const float kTopDownCameraDist = 5.0f;

	// Zoom in speed.
	const float kZoomSpeed = 10.0f;

	// Min/max clamp value of camera observation distance.
	const float kCamViewMinDist = 1.0f;
	const float kCamViewMaxDist = 100.f;

	// FOV set up values.
	// Third and top down camera's FOV is 65 degrees.
	// First person is color camera's FOV.
	const float kFov = 65.0f;

	// Scale frustum size for closer near clipping plane.
	const float kFovScaler = 0.1f;

	// Increment value each time move AR elements.
	const float kArElementIncrement = 0.05f;

	const float kZero = 0.0f;
	const glm::vec3 kZeroVec3 = glm::vec3(0.0f, 0.0f, 0.0f);
	const glm::quat kZeroQuat = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

	// AR grid rotation, 90 degrees around x axis.
	const glm::quat kArGridRotation = glm::quat(0.70711f, -0.70711f, 0.0f, 0.0f);
	const glm::quat kMarkerRotation = glm::quat(0.f, 0.f, 1.0f, 0.f);
	const glm::vec3 kMarkerPosition = glm::vec3(0.0f, 0.85f, -3.0f);
	const glm::vec3 kMarkerOffset = glm::vec3(0.0f, 0.85f, 0.0f);


	// Height offset is used for offset height of motion tracking
	// pose data. Motion tracking start position is (0,0,0). Adding
	// a height offset will give a more reasonable pose while a common
	// human is holding the device. The units is in meters.
	const glm::vec3 kFloorOffset = glm::vec3(0.0f, -1.4f, 0.0f);
	glm::vec3 world_position = glm::vec3(0.0f, -1.4f, 0.0f);

	// Position and rotation of the opengl camera with respect to the opengl world.
	// (This is the opengl representation of the physical color camera's location.)
	glm::vec3 ow_p_oc;
	glm::quat ow_q_oc;

	// Projection matrix from render camera.
	glm::mat4 projection_mat;

	// First person projection matrix from color camera intrinsics.
	glm::mat4 projection_mat_ar;

	// First person view matrix from color camera extrinsics.
	glm::mat4 view_mat;

	// Tango start-of-service with respect to Opengl World matrix.
	glm::mat4 ow_T_ss;

	// Device with respect to IMU matrix.
	glm::mat4 imu_T_device;

	// Color Camera with respect to IMU matrix.
	glm::mat4 imu_T_cc;

	// Opengl Camera with respect to Color Camera matrix.
	glm::mat4 cc_T_oc;

	// Opengl Camera with respect to Opengl World matrix.
	glm::mat4 ow_T_oc;

	// Color Camera image plane ratio.
	float image_plane_ratio;
	float image_width;
	float image_height;

	// Color Camera image plane distance to view point.
	float image_plane_dis;
	float image_plane_dis_original;

};

const int SKY_BOX_SIZE = 40;

static void onPoseAvailable(void*, const TangoPoseData* pose) {
  ci::app::console()<<"Position:"<<pose->translation[0]<<" " <<pose->translation[1]<< std::endl;
}


void CinderTangoApp::SetupExtrinsics() {
  CinderTango& instance = CinderTango::GetInstance();
  imu_T_device = glm::translate(glm::mat4(1.0f), instance.imu_p_device) *
                 glm::mat4_cast(instance.imu_q_device);

  imu_T_cc = glm::translate(glm::mat4(1.0f), instance.imu_p_cc) *
             glm::mat4_cast(instance.imu_q_cc);
}

// Setup projection matrix in first person view from color camera intrinsics.
void CinderTangoApp::SetupIntrinsics() {
  image_width = static_cast<float>(CinderTango::GetInstance().cc_width);
  image_height = static_cast<float>(CinderTango::GetInstance().cc_height);
  // Image plane focal length for x axis.
  float img_fl = static_cast<float>(CinderTango::GetInstance().cc_fx);
  image_plane_ratio = image_height / image_width;
  image_plane_dis_original = 2.0f * img_fl / image_width;
  image_plane_dis = image_plane_dis_original;
  projection_mat_ar = glm::frustum(
      -1.0f * kFovScaler, 1.0f * kFovScaler, -image_plane_ratio * kFovScaler,
      image_plane_ratio * kFovScaler, image_plane_dis * kFovScaler,
      kCamViewMaxDist);
  //frustum->SetScale(glm::vec3(1.0f, image_plane_ratio, image_plane_dis));
}
void CinderTangoApp::resize()
{
	mCam.setPerspective( kFov, getWindowAspectRatio(), .01, 100 );
}
void CinderTangoApp::setup()
{
	
	mGround = gl::Batch::create(geom::Cube().size(1,10,10), ci::gl::getStockShader(ci::gl::ShaderDef().color()));

    ow_T_ss = tango_gl::conversions::opengl_world_T_tango_world();
    cc_T_oc = tango_gl::conversions::color_camera_T_opengl_camera();


	gl::enableDepthRead();
	gl::enableDepthWrite();
	auto env = cinder::android::JniHelper::Get()->AttachCurrentThread();
	auto activity = cinder::android::app::CinderNativeActivity::getJavaObject();
	TangoErrorType err = CinderTango::GetInstance().Initialize(env, activity);
	if (err != TANGO_SUCCESS) {
	    if (err == TANGO_INVALID) {
	      ci::app::console()<<"Tango Service version mismatch"<<std::endl;
	    } else {
	      ci::app::console()<<"Tango Service initialize internal error"<<std::endl;
	    }
	}
	if (!CinderTango::GetInstance().SetConfig(true)) {
	   ci::app::console()<<"Tango set config failed"<<std::endl;
  	}
  	tangoConnected = false;

	if (!CinderTango::GetInstance().Connect()) {
	    ci::app::console()<<"Tango connect failed"<<std::endl;
	  }
	  else {
	  		CI_LOG_V("tango connected");
	  		tangoConnected = true;
			ci::gl::Texture2d::Format texFmt;
        	texFmt.target( GL_TEXTURE_EXTERNAL_OES );
        	texFmt.minFilter( GL_LINEAR );
        	texFmt.magFilter( GL_LINEAR );
        	texFmt.wrap( GL_CLAMP_TO_EDGE );
			mPassThru = gl::Texture2d::create( getWindowWidth(), getWindowHeight(), texFmt );
			CinderTango::GetInstance().ConnectTexture(mPassThru->getId());
	  }


}
void CinderTangoApp::update()
{

    if(tangoConnected){
    	CinderTango::GetInstance().UpdateColorTexture();
    	CinderTango::GetInstance().GetPoseAtTime();

  		glm::vec3 ss_p_device = CinderTango::GetInstance().tango_position;
	  glm::quat ss_q_device = CinderTango::GetInstance().tango_rotation;
	  glm::mat4 ss_T_device = glm::translate(glm::mat4(1.0f), ss_p_device) *
	                          glm::mat4_cast(ss_q_device);
	  ow_T_oc =
	      ow_T_ss * ss_T_device * glm::inverse(imu_T_device) * imu_T_cc * cc_T_oc;
	  glm::vec3 scale;
	  tango_gl::util::DecomposeMatrix(ow_T_oc, ow_p_oc, ow_q_oc, scale);
		projection_mat = projection_mat_ar;
    	view_mat = glm::inverse(ow_T_oc);
    	// Define what motion is requested.
            


            TangoCoordinateFramePair frames_of_reference;
            frames_of_reference.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
            frames_of_reference.target = TANGO_COORDINATE_FRAME_DEVICE;
            TangoPoseData pose;
    		TangoService_getPoseAtTime(0.0, frames_of_reference, &pose);
    		quat tangoPose = quat(pose.orientation[3], pose.orientation[0], pose.orientation[1], pose.orientation[2]);
    		const float M_SQRT_2_OVER_2 = sqrt(2) / 2.0f;
              glm::quat conversionQuaternion = glm::quat(M_SQRT_2_OVER_2, -M_SQRT_2_OVER_2,
                                                         0.0f, 0.0f);
             tangoPose = conversionQuaternion * tangoPose;
    		mCam.setOrientation(tangoPose);
			mCam.setEyePoint(vec3(pose.translation[0], pose.translation[1], pose.translation[2]));
			ci::app::console()<<"trans " << pose.translation[0] << " " << pose.translation[1] << " " << pose.translation[2] << std::endl;

    }
    //
	//mCam.setPerspective(60, getWindowAspectRatio(), 1, 1000);


}
void CinderTangoApp::mouseDrag( MouseEvent event )
{
	mPoints.push_back( event.getPos() );
}

#if defined( CINDER_ANDROID )
void CinderTangoApp::touchesBegan( TouchEvent event )
{
	for( vector<TouchEvent::Touch>::const_iterator touchIt = event.getTouches().begin(); touchIt != event.getTouches().end(); ++touchIt ) {
        mPoints.push_back( touchIt->getPos() );
	}
}
void CinderTangoApp::touchesMoved( TouchEvent event )
{
	for( vector<TouchEvent::Touch>::const_iterator touchIt = event.getTouches().begin(); touchIt != event.getTouches().end(); ++touchIt ) {
        mPoints.push_back( touchIt->getPos() );
	}
}
#endif

void CinderTangoApp::keyDown( KeyEvent event )
{
	if( event.getChar() == 'f' )
		setFullScreen( ! isFullScreen() );
}


void CinderTangoApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) );
	gl::setMatricesWindow(getWindowSize(),false);
		gl::pushMatrices();
		gl::disableDepthWrite();
    	gl::draw(mPassThru);
    	gl::popMatrices();
	gl::setMatrices( mCam );
	gl::enableDepthWrite();
	projection_mat = projection_mat_ar;
    view_mat = glm::inverse(ow_T_oc);

    gl::pushMatrices();
    //gl::setProjectionMatrix(projection_mat);
   // gl::setViewMatrix(view_mat);
    //gl::translate(world_position);
		//gl::multModelMatrix( mObjectRotation );
	gl::translate(0,-3,.5);
	//mGround->draw();
	gl::popMatrices();
	
	// draw sky box
	//gl::pushMatrices();
		//gl::scale( SKY_BOX_SIZE, SKY_BOX_SIZE, SKY_BOX_SIZE );
		//mSkyBoxBatch->draw();
	//gl::popMatrices();

}


// This line tells Cinder to actually create the application
CINDER_APP( CinderTangoApp, RendererGl )
