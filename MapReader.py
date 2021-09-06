# Map Reader copied from [MapWidget.py]
import json as js
class MapReader():
    def __init__(self, filename):
        self.map_name = filename
        self.js = dict()
        self.map_x = []
        self.map_y = []
        self.verts = []
        self.circles = []
        self.points = []
        self.straights = []
        self.res = -1
    # run method gets called when we start the thread
    def run(self):
        fid = open(self.map_name, encoding= 'UTF-8')
        self.js = js.load(fid)
        fid.close()
        self.map_x = []
        self.map_y = []
        self.verts = []
        self.circles = []
        self.straights = []
        self.points = []
        self.p_names = []
        self.res = self.js['header']['resolution']
        # print(self.js.keys())
        for pos in self.js['normalPosList']:
            if 'x' in pos:
                self.map_x.append(float(pos['x']))
            else:
                self.map_x.append(0.0)
            if 'y' in pos:
                self.map_y.append(float(pos['y']))
            else:
                self.map_y.append(0.0)
        if 'advancedCurveList' in self.js:
            for line in self.js['advancedCurveList']:
                if line['className'] == 'BezierPath':
                    x0 = 0
                    y0 = 0
                    x1 = 0
                    y1 = 0
                    x2 = 0
                    y2 = 0
                    x3 = 0
                    y3 = 0
                    if 'x' in line['startPos']['pos']:
                        x0 = line['startPos']['pos']['x']
                    if 'y' in line['startPos']['pos']:
                        y0 = line['startPos']['pos']['y']
                    if 'x' in line['controlPos1']:
                        x1 = line['controlPos1']['x']
                    if 'y' in line['controlPos1']:
                        y1 = line['controlPos1']['y']
                    if 'x' in line['controlPos2']:
                        x2 = line['controlPos2']['x']
                    if 'y' in line['controlPos2']:
                        y2 = line['controlPos2']['y']
                    if 'x' in line['endPos']['pos']:
                        x3 = line['endPos']['pos']['x']
                    if 'y' in line['endPos']['pos']:
                        y3 = line['endPos']['pos']['y']
                    self.verts.append([(x0,y0),(x1,y1),(x2,y2),(x3,y3)])
                elif line['className'] == 'ArcPath':
                    x1 = 0
                    y1 = 0
                    x2 = 0
                    y2 = 0
                    x3 = 0
                    y3 = 0
                    if 'x' in line['startPos']['pos']:
                        x1 = line['startPos']['pos']['x']
                    if 'y' in line['startPos']['pos']:
                        y1 = line['startPos']['pos']['y']
                    if 'x' in line['controlPos1']:
                        x2 = line['controlPos1']['x']
                    if 'y' in line['controlPos1']:
                        y2 = line['controlPos1']['y']
                    if 'x' in line['endPos']['pos']:
                        x3 = line['endPos']['pos']['x']
                    if 'y' in line['endPos']['pos']:
                        y3 = line['endPos']['pos']['y']
                    A = x1*(y2-y3) - y1*(x2-x3)+x2*y3-x3*y2
                    B = (x1*x1 + y1*y1)*(y3-y2)+(x2*x2+y2*y2)*(y1-y3)+(x3*x3+y3*y3)*(y2-y1)
                    C = (x1*x1 + y1*y1)*(x2-x3)+(x2*x2+y2*y2)*(x3-x1)+(x3*x3+y3*y3)*(x1-x2)
                    D = (x1*x1 + y1*y1)*(x3*y2-x2*y3)+(x2*x2+y2*y2)*(x1*y3-x3*y1)+(x3*x3+y3*y3)*(x2*y1-x1*y2)
                    if abs(A) > 1e-12:
                        x = -B/2/A
                        y = -C/2/A
                        r = math.sqrt((B*B+C*C-4*A*D)/(4*A*A))
                        theta1 = math.atan2(y1-y,x1-x)
                        theta3 = math.atan2(y3-y,x3-x)
                        v1 = np.array([x2-x1,y2-y1])
                        v2 = np.array([x3-x2,y3-y2])
                        flag = float(np.cross(v1,v2))
                        if flag >= 0:
                            self.circles.append([x, y, r, np.rad2deg(theta1), np.rad2deg(theta3)])
                        else:
                            self.circles.append([x, y, r, np.rad2deg(theta3), np.rad2deg(theta1)])
                    else:
                        self.straights.append([(x1,y1),(x3,y3)])
                elif line['className'] == 'StraightPath':
                    x1 = 0
                    y1 = 0
                    x2 = 0
                    y2 = 0
                    if 'x' in line['startPos']['pos']:
                        x1 = line['startPos']['pos']['x']
                    if 'y' in line['startPos']['pos']:
                        y1 = line['startPos']['pos']['y']
                    if 'x' in line['endPos']['pos']:
                        x2 = line['endPos']['pos']['x']
                    if 'y' in line['endPos']['pos']:
                        y2 = line['endPos']['pos']['y']
                    self.straights.append([(x1,y1),(x2,y2)])
        if 'advancedPointList' in self.js:
            for pt in self.js['advancedPointList']:
                x0 = 0
                y0 = 0 
                theta = 0
                if 'x' in pt['pos']:
                    x0 = pt['pos']['x']
                if 'y' in pt['pos']:
                    y0 = pt['pos']['y']
                if 'dir' in pt:
                    theta = pt['dir']
                if  'ignoreDir' in pt:
                    if pt['ignoreDir'] == True:
                        theta = None
                self.points.append([x0,y0,theta])
                self.p_names.append([pt['instanceName']])