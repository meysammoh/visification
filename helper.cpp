#include "helper.h"

void DrawCircle(float cx, float cy, float r, int num_segments)
{
    glBegin(GL_LINE_LOOP);
    for(int ii = 0; ii < num_segments; ii++)
    {
        float theta = 2.0f * 3.1415926f * float(ii) / float(num_segments);//get the current angle

        float x = r * cosf(theta);//calculate the x component
        float y = r * sinf(theta);//calculate the y component

        glVertex2f(x + cx, y + cy);//output vertex

    }
    glEnd();
}

double AngleBetweenThreePoints( Point_2 p1, Point_2 middlePoint, Point_2 p3 )
{
    double angle = 0;
    Vector_2 vect1, vect2;
    vect1 = p1 - middlePoint ;
    vect2 = p3 - middlePoint ;

    double len1 = sqrt( CGAL::to_double( vect1.squared_length() ) );
    double len2 = sqrt( CGAL::to_double( vect2.squared_length() ) );
    if( len1 != 0 )
    {
        vect1 = vect1 / len1;
    }
    if( len2 != 0 )
    {
        vect2 = vect2 / len2;
    }


    angle = atan2( CGAL::to_double( vect1.y() ), CGAL::to_double( vect1.x()) )
            - atan2( CGAL::to_double( vect2.y() ), CGAL::to_double( vect2.x() ) );
    if (angle < 0)
    {
        angle += 2 * M_PI;
    }

    return angle;
}
/**
 * @brief Chacks if a point is inside a semicircle (or fraction pie).
 * This function works only for sectors with an internal angle of <= 180 degrees.
 * @param radius
 * @param centerX
 * @param centerY
 * @param startAngle in radians
 * @param endAngle in radians
 * @param pointx
 * @param pointy
 * @return
*/
bool IsPointInsideSemiCircle( double radius, double centerX,double centerY,
                              double startAngle , double endAngle ,
                              double pointx  , double  pointy  )
{

    double radiusSquared;
    double startVectorX;
    double startVectorY;
    double endVectorX;
    double endVectorY;

    radiusSquared = radius * radius;
    startVectorX = cos(startAngle);
    startVectorY = sin(startAngle);
    endVectorX = cos(endAngle);
    endVectorY = sin(endAngle);
    double distanceX = pointx - centerX;
    double distanceY = pointy - centerY;
    if(((distanceX * distanceX) + (distanceY * distanceY)) > radiusSquared)
    {
        return false;
    }

    if(((distanceX * -startVectorY) + (distanceY * startVectorX)) < 0.0f)
    {
        return false;
    }

    if(((distanceX * -endVectorY) + (distanceY * endVectorX)) > 0.0f)
    {
        return false;
    }
    return true;
}

double Cross( Vector_2 vec1, Vector_2 vec2)
{
    return  CGAL::to_double( vec1.x() * vec2.y() - vec2.x() * vec1.y() );
}

void MakeTriangleCCW(Point_2& firstEndPoint, const Point_2 & middlePoint, Point_2& secondEndPoint )
{
    Vector_2 firstVector = firstEndPoint - middlePoint;
    Vector_2 secondVector = secondEndPoint - middlePoint;

    double firstSize = CGAL::to_double(firstVector.squared_length());
    double secondSize = CGAL::to_double(secondVector.squared_length());

    if( firstSize != 0 )
        firstVector = firstVector/sqrt( firstSize );
    if( secondSize != 0 )
        secondVector = secondVector/sqrt( secondSize );

    double cross = Cross(firstVector, secondVector );
    if( cross < 0 )
    {
        //We should swap first and second point
        Point_2 temp = firstEndPoint;
        firstEndPoint = secondEndPoint;
        secondEndPoint = temp;
    }
}

IntersectionType FindIntersection( Segment_2 first, Segment_2 second, CGAL::Object& intersection )
{
//    cout<<" Checking intersection of ("<<first<<") and ("<<second<<")\n";
    intersection = CGAL::intersection( first, second );

    Point_2 ipoint;
    Segment_2 iseg;
    if( CGAL::assign( ipoint, intersection ) )
    {
//        cout<<" Found point intersection: "<<ipoint<<endl;
        return IT_POINT;
    }
    else if ( CGAL::assign( iseg, intersection  ) )
    {
//        cout<<" Found segment intersection: "<<iseg<<endl;
        return IT_SEGMENT;
    }
    else
    {
//        cout<<" No intersection: "<<endl;
        return IT_NONE;
    }
}

Point_2 ExtendSegmentFromEnd( Point_2 segmentStart, Point_2 segmentEnd, double toSize, bool inOppositeDirection)
{
    //first point
//    cout<<" start "<<segmentStart<<" end "<<segmentEnd<<endl;
    double xdiff = CGAL::to_double( segmentEnd.x() - segmentStart.x() );
    double ydiff = CGAL::to_double( segmentEnd.y() - segmentStart.y() );
//    cout<<"xdiff "<<xdiff<<" ydiff "<<ydiff;
    if( inOppositeDirection )
    {
        xdiff *= -1;
        ydiff *= -1;
    }
//    cout<<"after direction: xdiff "<<xdiff<<" ydiff "<<ydiff;

    double len = sqrt( pow( xdiff, 2 ) +
                                  pow( ydiff, 2 ) );
//    cout<<"len "<<len<<endl;
    if( len > 0 )
    {
        double xShift = xdiff  * toSize  / len;
        double yShift = ydiff  * toSize / len;
//        cout<<"xshift "<<xShift<<"yshift "<<yShift<<endl;
        return Point_2( segmentStart.x() + xShift, segmentStart.y() + yShift );
    }
    else
    {
        cout<<"Warning: Length was zero!\n";
        return segmentStart;
    }

}

PointPlacementType CheckPointPlacement( Point_2 checkingPoint, Point_2 firstPoint, Point_2 secondPoint )
{
    //cout << "checking "<<checkingPoint<<" between first "<<firstPoint<<" second "<<secondPoint<<endl;
    Vector_2 firstToChecking = checkingPoint - firstPoint;
    Vector_2 firstToSecond = secondPoint - firstPoint ;
    double cross = Cross( firstToChecking, firstToSecond );

    if( cross != 0 )
    {
//        cout<<"OUT!\n";
        return PPT_OUT;
    }
    else
    {
        Vector_2 checkingToSecond = secondPoint - checkingPoint;
//        double f2sLen = sqrt( CGAL::to_double( secondToFirst.squared_length() ) );
        double f2cLen = sqrt( CGAL::to_double( firstToChecking.squared_length() ) );
        double c2sLen = sqrt( CGAL::to_double( checkingToSecond.squared_length() ) );
        if( (firstToChecking + checkingToSecond) == ( firstToSecond ) )
        {
            return PPT_BETWEEN;
        }
        else if( f2cLen > c2sLen )
        {
            return PPT_AFTER_SECOND;
        }
        else
        {
            return PPT_BEFORE_FIRST;
        }

    }
}
