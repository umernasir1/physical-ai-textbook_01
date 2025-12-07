from typing import Dict, Any, List


def get_personalization_recommendations(profile_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Generates personalization recommendations based on user profile data.

    Args:
        profile_data (Dict[str, Any]): A dictionary containing user-specific profile data,
                                       e.g., {"hardware": "rtx_4090", "experience_level": "beginner"}.

    Returns:
        Dict[str, Any]: A dictionary of personalization recommendations, e.g.,
                        {"content_level": "beginner", "hardware_focus": "GPU", "recommended_modules": ["basics", "cuda"]}.
    """
    recommendations = {
        "content_level": "general",
        "hardware_focus": "none",
        "recommended_modules": [],
        "explanation_style": "standard",
    }

    experience_level = profile_data.get("experience_level", "beginner").lower()
    hardware = profile_data.get("hardware", "").lower()

    if experience_level == "beginner":
        recommendations["content_level"] = "introductory"
        recommendations["explanation_style"] = "simple"
        recommendations["recommended_modules"].extend(
            ["introduction", "fundamental_concepts"]
        )
    elif experience_level == "intermediate":
        recommendations["content_level"] = "intermediate"
        recommendations["explanation_style"] = "detailed"
        recommendations["recommended_modules"].extend(
            ["advanced_algorithms", "practical_applications"]
        )
    elif experience_level == "expert":
        recommendations["content_level"] = "advanced"
        recommendations["explanation_style"] = "technical"
        recommendations["recommended_modules"].extend(
            ["research_topics", "optimization"]
        )

    if "rtx" in hardware or "gpu" in hardware or "cuda" in hardware:
        recommendations["hardware_focus"] = "GPU"
        if "cuda" not in recommendations["recommended_modules"]:
            recommendations["recommended_modules"].append("cuda_programming")
    elif "raspberry" in hardware or "edge" in hardware:
        recommendations["hardware_focus"] = "Edge AI"
        if "edge_deployment" not in recommendations["recommended_modules"]:
            recommendations["recommended_modules"].append("edge_deployment")

    # Remove duplicates from recommended_modules
    recommendations["recommended_modules"] = list(
        set(recommendations["recommended_modules"])
    )

    return recommendations


if __name__ == "__main__":
    # Example Usage
    user_profile_beginner_gpu = {"hardware": "RTX 4090", "experience_level": "Beginner"}
    recs_beginner_gpu = get_personalization_recommendations(user_profile_beginner_gpu)
    print("Beginner GPU User:", recs_beginner_gpu)

    user_profile_intermediate = {"experience_level": "Intermediate"}
    recs_intermediate = get_personalization_recommendations(user_profile_intermediate)
    print("Intermediate User:", recs_intermediate)

    user_profile_expert_edge = {
        "hardware": "Raspberry Pi",
        "experience_level": "Expert",
    }
    recs_expert_edge = get_personalization_recommendations(user_profile_expert_edge)
    print("Expert Edge AI User:", recs_expert_edge)
