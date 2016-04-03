String name="d";

void convert(int num) 
{
//  char c= char('a'+num);
//  String file= name+c;
  String file="thermobg";
  println(file);
  PImage img = loadImage(file+".png");  


  size( img.width, img.height );
  img.loadPixels();

  //println(img.width);
  //println(img.height);

  int length= img.width*img.height;
  int ptr=4;

  //WRITE
  byte[] output= new byte[2*length+4];
  output[0] = (byte)((img.width ) & 0xff); 
  output[1] = (byte)((img.width >>8) & 0xff);
  output[2] = (byte)((img.height ) & 0xff); 
  output[3] = (byte)((img.height >>8) & 0xff);


  for (int i=0; i<length; ++i) {
    color argb= img.pixels[i];
    int r = (argb >> 16) & 0xFF;  // Faster way of getting red(argb)
    int g = (argb >> 8) & 0xFF;   // Faster way of getting green(argb)
    int b = argb & 0xFF;          // Faster way of getting blue(argb)

    //fivesixfive
    int fsf = (r >> 3) & 0x1F;
    fsf <<= 6;
    fsf |= (g >> 2) & 0x3F;
    fsf <<= 5;
    fsf |= (b >> 3) & 0x1F;
    // low endian
    output[ptr++]= (byte)((fsf) & 0xFF);
    output[ptr++]= (byte)((fsf >> 8) & 0xFF);

    // Verification step
    r=(fsf>>11 & 0x1F)<<3;
    g=(fsf>>5 & 0x3F)<<2;
    b= (fsf & 0x1F)<<3;  
    img.pixels[i]=color(r, g, b);
  } 
  img.updatePixels();
  saveBytes(file+".fsf", output);

  //render
  image(img, 0, 0);
}

void read(String fname) {
  PImage img= createImage(50, 50, RGB);
  // READ
  byte[] input= loadBytes(fname+".fsf" );
  img.loadPixels();
  int ptr=4;
  for (int i=0; i<50*50; ++i) {
    // must be AND-ed to tell java to treat it as unsigned
    int fsf= (input[ptr+1] &0xFF ) << 8;
    fsf |= input[ptr] & 0xFF;

    ptr += 2;
//    println(hex(fsf));
    int r=(fsf>>11 & 0x1F)<<3;
    int g=(fsf>>5 & 0x3F)<<2;
    int b= (fsf & 0x1F)<<3;

    color argb= color(r & 0xFF, g & 0xFF, b & 0xFF);
    img.pixels[i]=argb;
  }
  img.updatePixels();
  
  //render
//  size( img.width, img.height );
//  image(img, 0, 0);  
}

void setup() {
//  noLoop();
//  for (int i=0; i<21; ++i) {
//    convert(i);
//  }
convert(-1);
  //size(50,50);
}

int n=0;
void draw() {
  if (n>20)
    return;
    
    String name="d"+char('a'+n);
     println(name); 
     read( name );
    // delay(100);
    n++;
  
}

