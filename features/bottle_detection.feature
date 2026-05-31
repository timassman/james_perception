Feature: Bottle detection from point cloud

  As a robot operator
  I want the bottle detector to find bottles in a 3D point cloud
  So that the robot knows where to pick them up

  Background:
    Given the bottle detector is configured with default parameters

  Scenario: Single bottle in an otherwise empty scene
    Given a point cloud loaded from "test_data/single_bottle.pcd"
    When I run the detector
    Then exactly 1 detection is returned
    And the detection has class_id "bottle"
    And the detection centroid is within 5 cm of the ground truth position

  Scenario: Two bottles side by side
    Given a point cloud loaded from "test_data/two_bottles.pcd"
    When I run the detector
    Then exactly 2 detections are returned
    And all detections have class_id "bottle"

  Scenario: No bottles present — background scene only
    Given a point cloud loaded from "test_data/empty_scene.pcd"
    When I run the detector
    Then 0 detections are returned

  Scenario: Bottle too close to the camera is ignored
    Given a point cloud loaded from "test_data/bottle_at_20cm.pcd"
    When I run the detector
    Then 0 detections are returned

  Scenario: Bottle beyond maximum range is ignored
    Given a point cloud loaded from "test_data/bottle_at_3m.pcd"
    When I run the detector
    Then 0 detections are returned

  Scenario: Bounding box size is plausible for a 500 ml bottle
    Given a point cloud loaded from "test_data/single_bottle.pcd"
    When I run the detector
    Then exactly 1 detection is returned
    And the bounding box height is between 15 cm and 35 cm
    And the bounding box width is between 4 cm and 12 cm
