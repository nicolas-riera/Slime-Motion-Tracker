#include "virtual_tracker.h"

namespace slimelite {

namespace {

vr::DriverPose_t emptyPose() {
	vr::DriverPose_t p{};
	p.qWorldFromDriverRotation.w = 1;
	p.qDriverFromHeadRotation.w = 1;
	p.qRotation.w = 1;
	p.result = vr::TrackingResult_Uninitialized;
	p.poseIsValid = false;
	p.deviceIsConnected = true;
	return p;
}

}  // namespace

VirtualTracker::VirtualTracker(std::string serial, std::string role)
	: m_serial(std::move(serial)), m_role(std::move(role)), m_pose(emptyPose()) {}

vr::EVRInitError VirtualTracker::Activate(uint32_t unObjectId) {
	m_id = unObjectId;
	auto props = vr::VRProperties();
	auto c = props->TrackedDeviceToPropertyContainer(m_id);
	props->SetStringProperty(c, vr::Prop_TrackingSystemName_String, "slimevr_lite");
	props->SetStringProperty(c, vr::Prop_ManufacturerName_String, "SlimeVR-lite");
	props->SetStringProperty(c, vr::Prop_ModelNumber_String, "SlimeVR-lite virtual tracker");
	props->SetStringProperty(c, vr::Prop_SerialNumber_String, m_serial.c_str());
	props->SetStringProperty(c, vr::Prop_RenderModelName_String, "{htc}vr_tracker_vive_1_0");
	props->SetStringProperty(c, vr::Prop_ControllerType_String, "slimevr_lite_tracker");
	props->SetStringProperty(c, vr::Prop_InputProfilePath_String,
							 "{slimevr_lite}/input/slimevr_lite_tracker_profile.json");
	props->SetInt32Property(c, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_OptOut);
	props->SetBoolProperty(c, vr::Prop_DeviceIsWireless_Bool, true);
	props->SetBoolProperty(c, vr::Prop_WillDriftInYaw_Bool, false);
	props->SetBoolProperty(c, vr::Prop_NeverTracked_Bool, false);
	props->SetBoolProperty(c, vr::Prop_Identifiable_Bool, false);
	return vr::VRInitError_None;
}

void VirtualTracker::Deactivate() { m_id = vr::k_unTrackedDeviceIndexInvalid; }

void VirtualTracker::updatePose(bool valid, const Pose& pose, const Vec3& velocity, bool sendVelocity) {
	if (!isActive()) return;
	vr::DriverPose_t p = emptyPose();
	if (valid) {
		p.poseIsValid = true;
		p.result = vr::TrackingResult_Running_OK;
		p.vecPosition[0] = pose.position.x;
		p.vecPosition[1] = pose.position.y;
		p.vecPosition[2] = pose.position.z;
		p.qRotation = {pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z};
		if (sendVelocity) {
			p.vecVelocity[0] = velocity.x;
			p.vecVelocity[1] = velocity.y;
			p.vecVelocity[2] = velocity.z;
		}
	} else {
		// Keep the last position so SteamVR does not snap the tracker to the origin
		p = m_pose;
		p.poseIsValid = false;
		p.result = vr::TrackingResult_Running_OutOfRange;
	}
	m_pose = p;
	vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_id, m_pose, sizeof(m_pose));
}

}  // namespace slimelite
