import React, { useState, useEffect } from 'react';
import { useAuth } from '@site/src/contexts/AuthContext';
import { personalizeContent } from '@site/src/utils/api-mock';

interface PersonalizeChapterButtonProps {
  chapterId: string;
  chapterTitle: string;
}

const PersonalizeChapterButton: React.FC<PersonalizeChapterButtonProps> = ({
  chapterId,
  chapterTitle
}) => {
  const { user, isLoading } = useAuth();
  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [personalizationApplied, setPersonalizationApplied] = useState(false);

  // Check if personalization has already been applied for this chapter
  useEffect(() => {
    const savedPersonalization = localStorage.getItem(`personalized-${chapterId}`);
    if (savedPersonalization) {
      setPersonalizationApplied(true);
    }
  }, [chapterId]);

  const handlePersonalize = async () => {
    if (!user) {
      alert('Please sign in to personalize chapters');
      return;
    }

    setIsPersonalizing(true);

    try {
      // Get the current chapter content
      const chapterElement = document.getElementById(`chapter-${chapterId}`);
      if (!chapterElement) {
        throw new Error('Chapter element not found');
      }

      const chapterContent = chapterElement.innerHTML;

      // Apply personalization based on user's background
      const personalizedContent = await personalizeContent(
        chapterContent,
        user.softwareBackground || 'general programming',
        user.hardwareBackground || 'general hardware'
      );

      // Apply personalization to the chapter content
      chapterElement.innerHTML = personalizedContent;

      // Save to localStorage that this chapter has been personalized
      localStorage.setItem(`personalized-${chapterId}`, 'true');
      setPersonalizationApplied(true);
    } catch (error) {
      console.error('Error personalizing chapter:', error);
      alert('Failed to personalize chapter. Please try again.');
    } finally {
      setIsPersonalizing(false);
    }
  };

  if (isLoading) {
    return <div>Loading...</div>;
  }

  if (!user) {
    return (
      <div style={{
        margin: '1rem 0',
        padding: '0.75rem',
        backgroundColor: '#f0f8ff',
        border: '1px solid #87ceeb',
        borderRadius: '4px',
        textAlign: 'center'
      }}>
        <p>Please <a href="/auth/signin">sign in</a> to personalize this chapter to your background.</p>
      </div>
    );
  }

  return (
    <div style={{
      margin: '1rem 0',
      padding: '0.75rem',
      backgroundColor: personalizationApplied ? '#e8f5e9' : '#fff3cd',
      border: `1px solid ${personalizationApplied ? '#4caf50' : '#ffc107'}`,
      borderRadius: '4px',
      textAlign: 'center'
    }}>
      <h3>Personalize Chapter</h3>
      <p>This chapter can be customized based on your background:</p>
      <p><strong>Software:</strong> {user.softwareBackground || 'Not specified'}</p>
      <p><strong>Hardware:</strong> {user.hardwareBackground || 'Not specified'}</p>

      <button
        onClick={handlePersonalize}
        disabled={isPersonalizing || personalizationApplied}
        style={{
          marginTop: '0.5rem',
          padding: '0.5rem 1rem',
          backgroundColor: personalizationApplied ? '#4caf50' : '#007cba',
          color: 'white',
          border: 'none',
          borderRadius: '4px',
          cursor: (isPersonalizing || personalizationApplied) ? 'not-allowed' : 'pointer',
        }}
      >
        {isPersonalizing
          ? 'Personalizing...'
          : personalizationApplied
            ? 'Chapter Personalized ✓'
            : 'Personalize This Chapter'}
      </button>
    </div>
  );
};

export default PersonalizeChapterButton;