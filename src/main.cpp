#include <Arduino.h>
#include <scoreboard.h>

TourneyMakerScoreboard *scoreboard = NULL;

class MyScoreReceivedCallback : public ScoreboardChangedCallback
{
  void onScoreReceived(uint8_t score1, uint8_t score2)
  {
    Serial.println("score received in callback " + String(score1) + ":" + String(score2));
  }

  void onColorReceived(uint32_t color1, uint32_t color2) {
    Serial.println("color received in callback " + String(color1) + ":" + String(color2));
  }
};

void setup()
{
  Serial.begin(9600);

  scoreboard = TourneyMakerScoreboard::setup("Papa");

  scoreboard->scoreboardChangedCallback = new MyScoreReceivedCallback();

}

void loop()
{
  Serial.print("loop");
  sleep(5);
  Serial.print("test");
  scoreboard->setScore(5, 0);
}