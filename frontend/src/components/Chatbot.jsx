import React, { useState } from 'react';
import { chat } from '../services/chat_api';

const Chatbot = () => {
  const [query, setQuery] = useState('');
  const [answer, setAnswer] = useState('');
  const [loading, setLoading] = useState(false);

  const handleQueryChange = (event) => {
    setQuery(event.target.value);
  };

  const handleSubmit = async (event) => {
    event.preventDefault();
    setLoading(true);
    try {
      const response = await chat({ query });
      setAnswer(response.answer);
    } catch (error) {
      setAnswer('Sorry, something went wrong.');
    }
    setLoading(false);
  };

  return (
    <div>
      <form onSubmit={handleSubmit}>
        <input type="text" value={query} onChange={handleQueryChange} />
        <button type="submit" disabled={loading}>
          {loading ? 'Loading...' : 'Ask'}
        </button>
      </form>
      {answer && <p>{answer}</p>}
    </div>
  );
};

export default Chatbot;
