package frc.robot.maths;


public class Vector2{
    public double x;
    public double y;
    public Vector2(double x, double y){
        this.x = x;
        this.y = y;
    }
    public static Vector2 fromPolar(double theta, double mag) {
        // n.b. theta = pi/2 - heading
        return new Vector2(mag*Math.cos(theta),mag*Math.sin(theta));
    }
    public Vector2 multiply(double scalar){
        return new Vector2(x*scalar,y*scalar);
    }
    public double dot(Vector2 v){
        return v.x * x + v.y * y;
    }
    public Vector2 add(Vector2 v){
        return new Vector2(v.x+x,v.y+y);
    }    
    public Vector2 proj(Vector2 v){
        return this.multiply(dot(v)/dot(this));                                                             
    }
    public double magnitude(){
        return Math.sqrt(x*x+y*y);
    }
    public double getAngle(Vector2 v) {
        return Math.acos(dot(v)/(dot(this)*v.dot(v)));
    }

}
