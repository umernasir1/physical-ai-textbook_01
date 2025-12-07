const API_URL = 'http://127.0.0.1:8000/api/v1/translation';

export const translateText = async (text, targetLanguage = 'ur') => {
    const response = await fetch(`${API_URL}/`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            text,
            target_language: targetLanguage,
        }),
    });

    const data = await response.json();
    if (!response.ok) {
        throw new Error(data.detail || 'Failed to translate text');
    }

    if (data.code !== 0) {
        throw new Error(data.message || 'An API error occurred');
    }

    return data.data;
};
