#include <glow/glutil.h>
#include <iostream>

#include "PCLCamera.h"

const Eigen::Matrix4f& PCLCamera::matrix() {
  mutex_.lock();

  // auto end = std::chrono::system_clock::now();
  // double dt = 0.0001 * std::chrono::duration_cast<std::chrono::milliseconds>(end - startTime_).count();

  // if (dt > 0 && startdrag_) {
  //   // apply velocity & reset timer...
  //   rotate(turnVel_ * dt, 0.0f);
  //   // translate(forwardVel_ * dt, upVel_ * dt, sideVel_ * dt);
  //   translate(sideVel_ * dt, forwardVel_ * dt, upVel_ * dt);
  //   startTime_ = end;
  // }

  // // recompute the view matrix (Euler angles) Remember: Inv(AB) = Inv(B)*Inv(A)
  // // Inv(translate*rotateYaw*rotatePitch) = Inv(rotatePitch)*Inv(rotateYaw)*Inv(translate)
  // view_ = glow::glRotateX(-pitch_);
  // view_ = view_ * glow::glRotateY(-yaw_);
  // view_ = view_ * glow::glTranslate(-x_, -y_, -z_);

  float x = distance_ * cos(yaw_) * cos(pitch_) + focal_point_.x();
  float y = distance_ * sin(yaw_) * cos(pitch_) + focal_point_.y();
  float z = distance_ * sin(pitch_) + focal_point_.z();

  mutex_.unlock();

  if (fabs(distance_) > 0.01) {
    lookAt(x, y, z, focal_point_.x(), focal_point_.y(), focal_point_.z());
  }

  return view_;
}

void PCLCamera::setMatrix(const Eigen::Matrix4f& m) {
  mutex_.lock();

  Eigen::Vector4f view_dir = m.transpose() * Eigen::Vector4f(0, 0, -1, 0);  // -Z direction
  Eigen::Vector4f new_cam = m.inverse() * Eigen::Vector4f(0, 0, 0, 1);

  // TODO: update focal_point
  if (view_dir[2] == 0) {
    focal_point_[0] = new_cam[0];
    focal_point_[1] = new_cam[1];
  } else {
    focal_point_[0] = new_cam[0] - new_cam[2] / view_dir[2] * view_dir[0];
    focal_point_[1] = new_cam[1] - new_cam[2] / view_dir[2] * view_dir[1];
  }

  distance_ = (focal_point_ - new_cam.block<3, 1>(0, 0)).norm();

  std::cout << "View Dir:" << view_dir << std::endl;
  std::cout << "New Cam:" << new_cam << std::endl;
  std::cout << "New Focal Point:" << focal_point_ << std::endl;

  mutex_.unlock();
  lookAt(new_cam.x(), new_cam.y(), new_cam.z(), focal_point_.x(), focal_point_.y(), focal_point_.z());
}

// since we are using here a different representation of the camera model we
// have to re-implement everything to allow an appropriate modification.
void PCLCamera::setPosition(float x, float y, float z) {
  mutex_.lock();
  x_ = x;
  y_ = y;
  z_ = z;

  mutex_.unlock();
}

Eigen::Vector4f PCLCamera::getPosition() const { return Eigen::Vector4f(x_, y_, z_, 1.0f); }

void PCLCamera::lookAt(float x_ref, float y_ref, float z_ref) {
  mutex_.lock();

  // float x = x_ref - x_;
  // float y = y_ref - y_;
  // float z = z_ref - z_;

  // Eigen::Vector3f dir(x, y, z);
  // dir.normalize();

  // // = std::acos(-dir.y()) - M_PI_2 = M_PI - std::acos(dir.y()) - M_PI_2; // in [-pi/2,pi/2]
  // pitch_ = std::asin(dir.y());
  // yaw_ = atan2(-x, -z);

  mutex_.unlock();
}

void PCLCamera::lookAt(float x_cam, float y_cam, float z_cam, float x_ref, float y_ref, float z_ref) {
  std::cout << "look at\n"
            << x_cam << "," << y_cam << "," << z_cam << "," << x_ref << "," << y_ref << "," << z_ref << std::endl;
  mutex_.lock();
  float x = x_ref - x_cam;
  float y = y_ref - y_cam;
  float z = z_ref - z_cam;

  Eigen::Vector3f dir(x, y, z);
  focal_point_ = Eigen::Vector3f(x_ref, y_ref, z_ref);
  distance_ = dir.norm();

  dir.normalize();

  // pitch_ = std::asin(dir.y());  // = std::acos(-dir.y()) - M_PI_2 in [-pi/2,pi/2]
  // yaw_ = atan2(-x, -z);
  Eigen::Vector3f tmp_up_dir(0, 0, 1.0);
  Eigen::Vector3f right_dir = tmp_up_dir.cross(-dir);
  // iff dir is paralle to Z direction, set right to -Y
  if (right_dir.norm() <= 0.01) {
    right_dir = -Eigen::Vector3f::UnitY();
  }
  Eigen::Vector3f up_dir = right_dir.cross(dir);
  std::cout << "Forward:\n" << dir.transpose() << std::endl;
  std::cout << "Right:\n" << right_dir.transpose() << std::endl;
  std::cout << "Up:\n" << up_dir.transpose() << std::endl;
  right_dir.normalize();
  up_dir.normalize();

  // rotation matrix to pitch yaw
  // Eigen::Matrix3f;
  Eigen::Matrix4f camera_pose = Eigen::Matrix4f::Identity();
  camera_pose(0, 3) = x_cam;
  camera_pose(1, 3) = y_cam;
  camera_pose(2, 3) = z_cam;
  camera_pose.block<3, 1>(0, 0) = right_dir;
  camera_pose.block<3, 1>(0, 1) = up_dir;
  camera_pose.block<3, 1>(0, 2) = -dir;
  std::cout << "camera_pose:\n" << camera_pose << std::endl;
  view_ = camera_pose.inverse();

  // calculate pitch yaw
  Eigen::Vector3f diff(x_cam - x_ref, y_cam - y_ref, z_cam - z_ref);
  pitch_ = std::asin(diff.z() / distance_);
  yaw_ = atan2(diff.y(), diff.x());

  mutex_.unlock();
  //  lookAt(x_ref, y_ref, z_ref);
}

bool PCLCamera::mousePressed(float x, float y, MouseButton btn, KeyboardModifier modifier) {
  std::cout << "Pressed\n" << x << "," << y << "," << yaw_ << "," << pitch_ << std::endl;
  startx_ = x;
  starty_ = y;
  last_x_ = x;
  last_y_ = y;
  startyaw_ = yaw_;
  startpitch_ = pitch_;
  if (btn == MouseButton::LeftButton) {
    startdrag_ = true;
  } else if (btn == MouseButton::MiddleButton) {
    start_move_ = true;
  }

  // std::cout<<static_cast<std::underlying_type<MouseButton>::type>(btn)<<std::endl;
  return true;
}

bool PCLCamera::mouseReleased(float x, float y, MouseButton btn, KeyboardModifier modifier) {
  startdrag_ = false;
  start_move_ = false;
  return true;
}

void PCLCamera::move(float sideways, float up, float backwards) {
  focal_point_ += view_.block<3, 3>(0, 0).inverse() * Eigen::Vector3f(sideways, up, backwards);
}

void PCLCamera::zoom(float amount) {
  distance_ -= amount;
  distance_ = std::max(0.1f, distance_);
}

/*****************************************************************************/

void PCLCamera::rotate(float yaw, float pitch) {
  yaw_ += yaw;
  pitch_ += pitch;
  if (pitch_ < -M_PI_2) pitch_ = -M_PI_2;
  if (pitch_ > M_PI_2) pitch_ = M_PI_2;
}

bool PCLCamera::mouseMoved(float x, float y, MouseButton btn, KeyboardModifier modifier) {
  mutex_.lock();

  // TODO: expose parameters:
  static const float MIN_MOVE = 0;
  //  static const float WALK_SENSITIVITY = 0.5f;
  //  static const float TURN_SENSITIVITY = 0.01f;
  //  static const float SLIDE_SENSITIVITY = 0.5f;

  //  static const float RAISE_SENSITIVITY = 0.5f;

  static const float LOOK_SENSITIVITY = 0.003f;
  static const float FREE_TURN_SENSITIVITY = 0.003f;

  float dx = x - startx_;
  float dy = y - starty_;
  float delta_x = x - last_x_;
  float delta_y = y - last_y_;
  if (dx > 0.0f) dx = std::max(0.0f, dx - MIN_MOVE);
  if (dx < 0.0f) dx = std::min(0.0f, dx + MIN_MOVE);
  if (dy > 0.0f) dy = std::max(0.0f, dy - MIN_MOVE);
  if (dy < 0.0f) dy = std::min(0.0f, dy + MIN_MOVE);

  // idea: if the velocity changes, we have to reset the start_time and update the camera parameters.
  if (btn == MouseButton::LeftButton) {
    yaw_ = startyaw_ - FREE_TURN_SENSITIVITY * dx;
    pitch_ = startpitch_ + LOOK_SENSITIVITY * dy;

    // ensure valid values.
    if (pitch_ < -M_PI_2) pitch_ = -M_PI_2;
    if (pitch_ > M_PI_2) pitch_ = M_PI_2;
    std::cout << "Yaw:" << yaw_ << "\tPitch:" << pitch_ << std::endl;
    std::cout << "dx dy:" << dx << "," << dy << "," << startpitch_ << std::endl;
  } else if (btn == MouseButton::MiddleButton && start_move_) {
    // // translate(TURN_SENSITIVITY * dx,0,TURN_SENSITIVITY * dy);
    // float sideways = SLIDEX_SENSITIVITY * dx * (-1);
    // float forward = SLIDEY_SENSITIVITY * dy * std::sin(pitch_) * (-1);
    // float up = SLIDEU_SENSITIVITY * dy * std::cos(pitch_);
    // float s = std::sin(yaw_);
    // float c = std::cos(yaw_);

    // float factor = (startcy_ > 50 ? 25 : (startcy_ > 1 ? startcy_ * 0.5 : .5));
    // float upfactor = (startcy_ > 10 ? 5 : (startcy_ > 1 ? startcy_ * 0.5 : .5));

    // // std::cout<<x_<<"\t"<<y_<<"\t"<<z_<<"\t"<<factor<<std::endl;

    // x_ = startcx_ + (sideways * c - forward * s) * factor;
    // y_ = startcy_ + up * upfactor;
    // z_ = startcz_ - (sideways * s - forward * c * (-1)) * factor;

    static const float SLIDE_SENSITIVITY = 0.005f;

    move(-SLIDE_SENSITIVITY * delta_x, SLIDE_SENSITIVITY * delta_y, 0);
  } else if (btn == MouseButton::RightButton) {
    if (modifier != KeyboardModifier::ShiftDown) {
      static const float ZOOM_SENSITIVITY = 0.06f;
      zoom(ZOOM_SENSITIVITY * delta_y);
    } else if (modifier == KeyboardModifier::ShiftDown) {
      static const float FORWARD_SENSITIVITY = 0.03f;
      move(0, 0, FORWARD_SENSITIVITY * delta_y);
    }
  }

  last_x_ = x;
  last_y_ = y;

  mutex_.unlock();

  return true;
}

bool PCLCamera::wheelEvent(float delta, KeyboardModifier modifier) {
  mutex_.lock();
  static const float ZOOM_SENSITIVITY = 3.f;
  // move along the viewing direction specified by yaw and pitch.
  // // TODO: implement me!
  zoom(ZOOM_SENSITIVITY * delta);
  mutex_.unlock();

  return true;
}

bool PCLCamera::keyPressed(KeyboardKey key, KeyboardModifier modifier) {
  float factor = (y_ > 50 ? 50 : (y_ > 1 ? y_ : 1));
  switch (key) {
    case KeyboardKey::KeyA:
      startTime_ = std::chrono::system_clock::now();
      startdrag_ = true;
      sideVel_ = -10 * factor;
      return true;
    case KeyboardKey::KeyD:
      startTime_ = std::chrono::system_clock::now();
      startdrag_ = true;
      sideVel_ = 10 * factor;
      return true;
    case KeyboardKey::KeyW:
      startTime_ = std::chrono::system_clock::now();
      startdrag_ = true;
      forwardVel_ = 10 * factor;
      return true;
    case KeyboardKey::KeyS:
      startTime_ = std::chrono::system_clock::now();
      startdrag_ = true;
      forwardVel_ = -10 * factor;
      return true;
    default:
      return false;
  }
}

bool PCLCamera::keyReleased(KeyboardKey key, KeyboardModifier modifier) {
  switch (key) {
    case KeyboardKey::KeyA:
    case KeyboardKey::KeyD:
      startTime_ = std::chrono::system_clock::now();

      sideVel_ = 0;
      if (forwardVel_ == 0) startdrag_ = false;
      return true;
    case KeyboardKey::KeyW:
    case KeyboardKey::KeyS:
      startTime_ = std::chrono::system_clock::now();

      forwardVel_ = 0;
      if (sideVel_ == 0) startdrag_ = false;
      return true;
    default:
      return false;
  }
}

/* namespace rv */
