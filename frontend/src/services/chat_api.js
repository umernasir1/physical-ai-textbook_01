const API_URL = 'http://localhost:8000'; // Assuming the backend is running on port 8000

export const chat = async (request) => {
  const response = await fetch(`${API_URL}/api/v1/chat`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(request),
  });
  const data = await response.json();
  if (!response.ok) {
    throw new Error(data.detail || data.message || 'Failed to fetch');
  }
  if (data.code !== 0) {
      throw new Error(data.message || 'An API error occurred');
  }
  return data.data;
};
