cmake_minimum_required(VERSION 3.5)
project(romea_mobile_base_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch ros2_control urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_module(python/romea_mobile_base_description.py)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cflake8)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  configure_file(test/test_mobile_base_parameters_1FAS2RWD.yaml  test/test_mobile_base_parameters_1FAS2RWD.yaml)
  configure_file(test/test_mobile_base_parameters_1FAS2FWD.yaml  test/test_mobile_base_parameters_1FAS2FWD.yaml)
  configure_file(test/test_mobile_base_parameters_1FAS4WD.yaml  test/test_mobile_base_parameters_1FAS4WD.yaml)
  configure_file(test/test_mobile_base_parameters_2AS4WD.yaml  test/test_mobile_base_parameters_2AS2FWD.yaml)
  configure_file(test/test_mobile_base_parameters_2AS4WD.yaml  test/test_mobile_base_parameters_2AS2RWD.yaml)
  configure_file(test/test_mobile_base_parameters_2AS4WD.yaml  test/test_mobile_base_parameters_2AS4WD.yaml)
  configure_file(test/test_mobile_base_parameters_2FWS2FWD.yaml  test/test_mobile_base_parameters_2FWS2FWD.yaml)
  configure_file(test/test_mobile_base_parameters_2FWS2RWD.yaml  test/test_mobile_base_parameters_2FWS2RWD.yaml)
  configure_file(test/test_mobile_base_parameters_2FWS4WD.yaml  test/test_mobile_base_parameters_2FWS4WD.yaml)
  configure_file(test/test_mobile_base_parameters_2TD.yaml  test/test_mobile_base_parameters_2TD.yaml)
  configure_file(test/test_mobile_base_parameters_2WD.yaml  test/test_mobile_base_parameters_2WD.yaml)
  configure_file(test/test_mobile_base_parameters_4WD.yaml  test/test_mobile_base_parameters_4WD.yaml)
  configure_file(test/test_mobile_base_parameters_4WS4WD.yaml  test/test_mobile_base_parameters_4WS4WD.yaml)
  ament_add_pytest_test(test_mobile_base_description test/test_mobile_base_description.py)

endif()

ament_package()
