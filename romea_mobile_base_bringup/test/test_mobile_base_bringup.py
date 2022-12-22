import os
import pytest

from romea_mobile_base_bringup import MobileBaseMetaDescription


@pytest.fixture(scope="module")
def meta_description():
    meta_description_filename = os.path.join(
        os.getcwd(), "test_mobile_base_bringup.yaml"
    )
    return MobileBaseMetaDescription(meta_description_filename)


def test_get_name(meta_description):
    assert meta_description.get_name() == "adap2e"


def test_get_type(meta_description):
    assert meta_description.get_type() == "adap2e"


def test_get_model(meta_description):
    assert meta_description.get_model() == "fat"


def test_get_simulation_initial_xyz(meta_description):
    assert meta_description.get_simulation_initial_xyz() == [1.0, 2.0, 3.0]


def test_get_simulation_initial_rpy(meta_description):
    assert meta_description.get_simulation_initial_rpy() == [4.0, 5.0, 6.0]
