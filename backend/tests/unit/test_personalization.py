import pytest
from typing import Dict, Any

from backend.src.core.personalization import get_personalization_recommendations


def test_get_personalization_recommendations_empty_profile():
    """
    Test recommendations for an empty profile data.
    """
    profile_data: Dict[str, Any] = {}
    recommendations = get_personalization_recommendations(profile_data)

    assert recommendations["content_level"] == "introductory"
    assert recommendations["hardware_focus"] == "none"
    assert "introduction" in recommendations["recommended_modules"]
    assert "fundamental_concepts" in recommendations["recommended_modules"]
    assert recommendations["explanation_style"] == "simple"


def test_get_personalization_recommendations_beginner_gpu():
    """
    Test recommendations for a beginner user with GPU hardware.
    """
    profile_data = {"experience_level": "Beginner", "hardware": "RTX 3080"}
    recommendations = get_personalization_recommendations(profile_data)

    assert recommendations["content_level"] == "introductory"
    assert recommendations["hardware_focus"] == "GPU"
    assert "introduction" in recommendations["recommended_modules"]
    assert "fundamental_concepts" in recommendations["recommended_modules"]
    assert "cuda_programming" in recommendations["recommended_modules"]
    assert recommendations["explanation_style"] == "simple"


def test_get_personalization_recommendations_intermediate_edge_ai():
    """
    Test recommendations for an intermediate user with Edge AI hardware.
    """
    profile_data = {"experience_level": "Intermediate", "hardware": "Raspberry Pi 4"}
    recommendations = get_personalization_recommendations(profile_data)

    assert recommendations["content_level"] == "intermediate"
    assert recommendations["hardware_focus"] == "Edge AI"
    assert "advanced_algorithms" in recommendations["recommended_modules"]
    assert "practical_applications" in recommendations["recommended_modules"]
    assert "edge_deployment" in recommendations["recommended_modules"]
    assert recommendations["explanation_style"] == "detailed"


def test_get_personalization_recommendations_expert_no_hardware():
    """
    Test recommendations for an expert user with no specific hardware mentioned.
    """
    profile_data = {"experience_level": "Expert"}
    recommendations = get_personalization_recommendations(profile_data)

    assert recommendations["content_level"] == "advanced"
    assert recommendations["hardware_focus"] == "none"
    assert "research_topics" in recommendations["recommended_modules"]
    assert "optimization" in recommendations["recommended_modules"]
    assert recommendations["explanation_style"] == "technical"


def test_get_personalization_recommendations_case_insensitivity():
    """
    Test that hardware and experience level matching is case-insensitive.
    """
    profile_data = {"experience_level": "BEGINNER", "hardware": "rtx 2060"}
    recommendations = get_personalization_recommendations(profile_data)

    assert recommendations["content_level"] == "introductory"
    assert recommendations["hardware_focus"] == "GPU"
    assert "cuda_programming" in recommendations["recommended_modules"]

    profile_data_2 = {"experience_level": "eXpErt", "hardware": "EDGe device"}
    recommendations_2 = get_personalization_recommendations(profile_data_2)
    assert recommendations_2["content_level"] == "advanced"
    assert recommendations_2["hardware_focus"] == "Edge AI"
    assert "edge_deployment" in recommendations_2["recommended_modules"]
