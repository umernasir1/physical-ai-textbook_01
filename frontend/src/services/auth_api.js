const API_URL = 'http://127.0.0.1:8000/api/v1/auth';

export const login = async (email, password) => {
    const response = await fetch(`${API_URL}/login`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/x-www-form-urlencoded',
        },
        body: new URLSearchParams({
            username: email,
            password: password,
        }),
    });

    const data = await response.json();
    if (!response.ok) {
        throw new Error(data.detail || 'Failed to login');
    }

    if (data.code !== 0) {
        throw new Error(data.message || 'An API error occurred');
    }

    return data.data.access_token;
};

export const signup = async (email, password, profileData) => {
    const response = await fetch(`${API_URL}/signup`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            email,
            password,
            profile_data: profileData,
        }),
    });

    const data = await response.json();
    if (!response.ok) {
        throw new Error(data.detail || 'Failed to sign up');
    }

    if (data.code !== 0) {
        throw new Error(data.message || 'An API error occurred');
    }

    return data.data;
};