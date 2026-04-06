// DETECTION MODULE - detection_node.py
// Vision-based human detection using YOLOv8
// Filters detections temporally before publishing to FSM

PARAMETERS:
  uav_id
  model_path          // path to yolo weights
  confidence_threshold  // minimum confidence e.g. 0.5
  persistence_threshold // consecutive frames required e.g. 3

SUBSCRIPTIONS:
  /{uav_id}/camera/image    → on_image_received()
  /uav/state                → on_state_change()  // know when VERIFYING

PUBLICATIONS:
  /{uav_id}/fsm/event       → FSMEvent.msg (DETECTION_EVENT, CONFIRMED_TARGET, FALSE_POSITIVE)

STATE:
  model = YOLO(model_path)
  consecutive_detections = 0
  current_state = IDLE      // mirrors FSM, know when VERIFYING
  last_detection = None     // location of last detection

// ==============================
// Image Processing
// ==============================

on_image_received(msg):
  if current_state not in [SEARCHING, REFINING, ASSISTING, VERIFYING]:
    return  // don't process if not actively searching

  img = convert msg to cv2 image
  results = model(img, classes=[0])  // class 0 = person

  detections = filter results by confidence_threshold

  if detections is empty:
    consecutive_detections = 0
    if current_state == VERIFYING:
      // nothing seen during verification
      publish → /{uav_id}/fsm/event
      event: FALSE_POSITIVE
    return

  // take highest confidence detection
  best = detection with highest confidence
  last_detection = best.location
  consecutive_detections += 1

  if consecutive_detections >= persistence_threshold:
    if current_state == VERIFYING:
      publish → /{uav_id}/fsm/event
      event: CONFIRMED_TARGET, value: confidence
    else:
      publish → /{uav_id}/fsm/event
      event: DETECTION_EVENT, value: confidence
    consecutive_detections = 0  // reset after publishing

// ==============================
// State Tracking
// ==============================

on_state_change(UAVState msg):
  current_state = msg.state
  
  // reset on new search state
  if current_state == SEARCHING:
    consecutive_detections = 0
    last_detection = None