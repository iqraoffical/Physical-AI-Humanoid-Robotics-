import React from 'react';
import PersonalizeChapterButton from '@site/src/components/PersonalizeChapterButton';
import TranslateToUrduButton from '@site/src/components/TranslateToUrduButton';

interface ChapterActionsProps {
  chapterId: string;
  chapterTitle: string;
}

const ChapterActions: React.FC<ChapterActionsProps> = ({ chapterId, chapterTitle }) => {
  return (
    <div style={{ 
      display: 'flex', 
      flexDirection: 'column' as const, 
      gap: '1rem',
      marginBottom: '2rem',
      padding: '1rem',
      backgroundColor: '#f8f9fa',
      borderRadius: '8px',
      border: '1px solid #e9ecef'
    }}>
      <h3>Chapter Tools</h3>
      <PersonalizeChapterButton chapterId={chapterId} chapterTitle={chapterTitle} />
      <TranslateToUrduButton chapterId={chapterId} />
    </div>
  );
};

export default ChapterActions;