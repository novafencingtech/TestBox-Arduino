class GFXcanvas16stack : public GFXcanvas16 {
public:
  // Constructor
  GFXcanvas16stack(uint16_t* memPointer) :  {
    buffer=memPointer;
  }

private:
  // Private members specific to this subclass
  uint16_t my_custom_data;
};