const API_URL = 'http://localhost:8000'; // Assuming the backend is running on port 8000

export const chat = async ({ query, selectedText }) => {
  const chatText = selectedText || query;

  const response = await fetch(`${API_URL}/api/v1/chat`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({ text: chatText }),
  });

  if (!response.ok) {
    const errorData = await response.json().catch(() => ({}));
    throw new Error(errorData.detail || errorData.message || 'Failed to fetch chat response');
  }

  const data = await response.json();
  return { answer: data.response };
};
