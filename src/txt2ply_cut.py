import os
#from IPython.core.debugger import Tracer

class TXTConverter:
  def __init__(self, input_txt_filename, output_ply_filename):
    self.input_txt_filename = input_txt_filename
    self.output_ply_filename = output_ply_filename
    self.ply_header = ('ply\n'
        'format ascii 1.0\n'
        'element vertex %d\n'
        'property float x\n'
        'property float y\n'
        'property float z\n'
        'end_header\n')
              
  def convert(self):
    with open(self.input_txt_filename, 'a+') as fin, open(self.output_ply_filename, 'w') as fout:
      lines = fin.readlines()
      i=0
      #fout.write(self.ply_header % (len(lines)-1))
      for line in lines[:-1]:

        x, y, z = line.strip().split(',')
        if float(y)>2:
            print(y)
            i+=1
            print(i)
            out_str = '%f %f %f \n' % (float(x), float(y), float(z))
            fout.write(out_str)
      print(self.output_ply_filename)
      fout.seek(0, 0)
      fout.write(self.ply_header % (i))

if __name__ == '__main__':
    #if len(sys.argv) is not 3:
        #print("Usage: python txt2ply.py input_txt.txt output_ply.ply")
        #sys.exit(-1)
    sourcepath = "/home/huo/opencv_code/PPF/src/pointtxt/"
    targetpath = "/home/huo/opencv_code/PPF/src/pointply/"
    all_file = os.listdir(sourcepath)
    for files in all_file:
        outfile = os.path.splitext(files)[0] + ".ply"
        c = TXTConverter(sourcepath + files, targetpath + outfile)
        c.convert()
