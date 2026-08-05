document.querySelectorAll('[data-quiz]').forEach((quiz) => {
  const answer = quiz.dataset.answer;
  const explanation = quiz.dataset.explanation;
  const feedback = quiz.querySelector('.feedback');

  quiz.querySelectorAll('button[data-choice]').forEach((button) => {
    button.addEventListener('click', () => {
      quiz.querySelectorAll('button[data-choice]').forEach((item) => {
        item.classList.remove('correct', 'wrong');
      });

      if (button.dataset.choice === answer) {
        button.classList.add('correct');
        feedback.textContent = `正确。${explanation}`;
        feedback.className = 'feedback ok';
      } else {
        button.classList.add('wrong');
        feedback.textContent = '还不对。先回忆它负责的是应用、通知、锁还是时钟，再试一次。';
        feedback.className = 'feedback retry';
      }
    });
  });
});
