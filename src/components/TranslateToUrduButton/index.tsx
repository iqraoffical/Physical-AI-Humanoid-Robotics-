import React, { useState } from 'react';
import { translateToUrdu } from '@site/src/utils/api-mock';

interface TranslateToUrduButtonProps {
  chapterId: string;
}

const TranslateToUrduButton: React.FC<TranslateToUrduButtonProps> = ({ chapterId }) => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [translationApplied, setTranslationApplied] = useState(false);

  const handleTranslate = async () => {
    setIsTranslating(true);

    try {
      // Get the chapter content
      const chapterElement = document.getElementById(`chapter-${chapterId}`);
      if (!chapterElement) {
        throw new Error('Chapter element not found');
      }

      const chapterContent = chapterElement.innerHTML;

      // Translate to Urdu
      const translatedContent = await translateToUrdu(chapterContent);

      // Apply the translated content to the chapter
      chapterElement.innerHTML = translatedContent;
      setTranslationApplied(true);
    } catch (error) {
      console.error('Error translating chapter:', error);
      alert('Failed to translate chapter. Please try again.');
    } finally {
      setIsTranslating(false);
    }
  };

  return (
    <div style={{
      margin: '1rem 0',
      padding: '0.75rem',
      backgroundColor: translationApplied ? '#e8f5e9' : '#fff3cd',
      border: `1px solid ${translationApplied ? '#4caf50' : '#ffc107'}`,
      borderRadius: '4px',
      textAlign: 'center'
    }}>
      <button
        onClick={handleTranslate}
        disabled={isTranslating || translationApplied}
        style={{
          padding: '0.5rem 1rem',
          backgroundColor: translationApplied ? '#4caf50' : '#ff9800',
          color: 'white',
          border: 'none',
          borderRadius: '4px',
          cursor: (isTranslating || translationApplied) ? 'not-allowed' : 'pointer',
        }}
      >
        {isTranslating
          ? 'Translating...'
          : translationApplied
            ? 'Content in Urdu ✓'
            : '.Translate to Urdu'}
      </button>

      {translationApplied && (
        <p style={{ marginTop: '0.5rem', fontSize: '0.9rem', color: '#666' }}>
          Chapter content has been translated to Urdu
        </p>
      )}
    </div>
  );
};

export default TranslateToUrduButton;