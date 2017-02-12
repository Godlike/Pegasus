#ifndef PEGAS_DEMO_APP_HPP
#define PEGAS_DEMO_APP_HPP

class Application {
protected:
  int height;
  int width;

public:
  virtual const char *getTitle();

  virtual void initGraphics();

  virtual void setView();

  virtual void deinit();

  virtual void display();

  virtual void update();

  virtual void key(unsigned char key);

  virtual void resize(int width, int height);

  virtual void mouse(int button, int state, int x, int y);

  virtual void mouseDrag(int x, int y);

  void renderText(float x, float y, const char *text, void *font = nullptr);
};

#endif // PEGAS_DEMO_APP_HPP
