#include "virtual_tracker.h"

namespace smt {

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
	props->SetStringProperty(c, vr::Prop_TrackingSystemName_String, "slime_motion_tracker");
	props->SetStringProperty(c, vr::Prop_ManufacturerName_String, "Slime Motion Tracker");
	props->SetStringProperty(c, vr::Prop_ModelNumber_String, "Slime Motion Tracker virtual tracker");
	props->SetStringProperty(c, vr::Prop_SerialNumber_String, m_serial.c_str());
	props->SetStringProperty(c, vr::Prop_RenderModelName_String, "{htc}vr_tracker_vive_1_0");
	props->SetStringProperty(c, vr::Prop_ControllerType_String, "slime_motion_tracker_tracker");
	props->SetStringProperty(c, vr::Prop_InputProfilePath_String,
							 "{slime_motion_tracker}/input/slime_motion_tracker_tracker_profile.json");
	props->SetInt32Property(c, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_OptOut);
	props->SetBoolProperty(c, vr::Prop_DeviceIsWireless_Bool, true);
	props->SetBoolProperty(c, vr::Prop_WillDriftInYaw_Bool, false);
	props->SetBoolProperty(c, vr::Prop_NeverTracked_Bool, false);
	props->SetBoolProperty(c, vr::Prop_Identifiable_Bool, false);
	// Status icons, drawn by tools/icon/make_tracker_icons.py
	const struct {
		vr::ETrackedDeviceProperty prop;
		const char* path;
	} icons[] = {
		{vr::Prop_NamedIconPathDeviceOff_String, "{slime_motion_tracker}/icons/tracker_off.png"},
		{vr::Prop_NamedIconPathDeviceSearching_String, "{slime_motion_tracker}/icons/tracker_searching.png"},
		{vr::Prop_NamedIconPathDeviceSearchingAlert_String,
		 "{slime_motion_tracker}/icons/tracker_searching_alert.png"},
		{vr::Prop_NamedIconPathDeviceReady_String, "{slime_motion_tracker}/icons/tracker_ready.png"},
		{vr::Prop_NamedIconPathDeviceReadyAlert_String, "{slime_motion_tracker}/icons/tracker_ready_alert.png"},
		{vr::Prop_NamedIconPathDeviceNotReady_String, "{slime_motion_tracker}/icons/tracker_not_ready.png"},
		{vr::Prop_NamedIconPathDeviceStandby_String, "{slime_motion_tracker}/icons/tracker_standby.png"},
		{vr::Prop_NamedIconPathDeviceStandbyAlert_String,
		 "{slime_motion_tracker}/icons/tracker_standby_alert.png"},
		{vr::Prop_NamedIconPathDeviceAlertLow_String, "{slime_motion_tracker}/icons/tracker_alert_low.png"},
	};
	for (const auto& icon : icons) props->SetStringProperty(c, icon.prop, icon.path);
	return vr::VRInitError_None;
}

void VirtualTracker::Deactivate() { m_id = vr::k_unTrackedDeviceIndexInvalid; }

void VirtualTracker::updateBattery(const SmtBattery& battery) {
	if (!isActive()) return;
	auto props = vr::VRProperties();
	auto c = props->TrackedDeviceToPropertyContainer(m_id);
	props->SetBoolProperty(c, vr::Prop_DeviceProvidesBatteryStatus_Bool, battery.known != 0);
	if (!battery.known) return;
	props->SetFloatProperty(c, vr::Prop_DeviceBatteryPercentage_Float, battery.level);
	props->SetBoolProperty(c, vr::Prop_DeviceIsCharging_Bool, battery.charging != 0);
}

void VirtualTracker::updatePose(const SmtPose& pose, const SmtDeviceStatus& status, bool sendVelocity) {
	if (!isActive()) return;
	vr::DriverPose_t p = emptyPose();
	if (status.connected && pose.valid) {
		p.poseIsValid = true;
		p.result = vr::TrackingResult_Running_OK;
		for (int i = 0; i < 3; i++) p.vecPosition[i] = pose.position[i];
		p.qRotation = {pose.rotation[0], pose.rotation[1], pose.rotation[2], pose.rotation[3]};
		if (sendVelocity)
			for (int i = 0; i < 3; i++) p.vecVelocity[i] = pose.velocity[i];
	} else {
		// Keep the last position so SteamVR does not snap the tracker to the origin
		p = m_pose;
		for (int i = 0; i < 3; i++) p.vecVelocity[i] = 0;
		p.poseIsValid = false;
		p.deviceIsConnected = status.connected != 0;
		if (!status.connected)
			p.result = vr::TrackingResult_Uninitialized;  // shown off
		else if (!status.calibrated)
			p.result = vr::TrackingResult_Calibrating_OutOfRange;  // waiting for a calibration
		else
			p.result = vr::TrackingResult_Running_OutOfRange;  // briefly: heading, HMD lost...
	}
	m_pose = p;
	vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_id, m_pose, sizeof(m_pose));
}

}  // namespace smt
