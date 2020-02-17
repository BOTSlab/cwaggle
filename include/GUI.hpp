#pragma once

#include <SFML/Graphics.hpp>
#include <sstream>
#include <iostream>

#include "Vec2.hpp"
#include "Simulator.hpp"
#include "ExampleWorlds.hpp"
#include "SensorTools.hpp"

typedef void (*callback_function)(void);

class GUI
{
    std::shared_ptr<Simulator> m_sim;
    sf::RenderWindow    m_window;           // the window we will draw to
    sf::Font            m_font;             // the font we will use to draw
    sf::Text            m_text;
    sf::Clock           m_clock;
    sf::Vector2f        m_mousePos;
    Entity              m_selected;
    bool                m_debug = false;
    bool                m_sensors = false;
    std::string         m_status = "";
    bool                m_leftMouseDown = false;
    
    //std::vector<sf::RectangleShape> m_gridRectangles;

    // AV: For drawing a background image.
    sf::Texture         m_backgroundTexture;
    sf::Sprite          m_backgroundSprite;

    // AV: To change the background image we set this
    // pointer.  A NULL value means nothing will be
    // drawn on the background.
    sf::Image*          m_backgroundImagePtr;

    std::vector<sf::Image>          m_gridImages;

    // AV: For showing contour lines of the grid.
    sf::Image           m_contourImage;

    // AV: For showing the occupancy of robots
    sf::Image           m_occupancyImage;

    callback_function   m_upArrowCallback;
    callback_function   m_downArrowCallback;
    callback_function   m_spaceCallback;

    void init(std::shared_ptr<Simulator> sim)
    {
        m_sim = sim;
        m_font.loadFromFile("fonts/cour.ttf");
        m_text.setFont(m_font);
        m_text.setCharacterSize(24);
        m_text.setPosition(5, 5);
        //m_text.setFillColor(sf::Color::Yellow);

        int width = m_sim->getWorld()->width();
        int height = m_sim->getWorld()->height();

        // Create all images which can be used as the background.
        for (int i=0; i< m_sim->getWorld()->getNumberOfGrids(); i++) {
            auto & grid = m_sim->getWorld()->getGrid(i);
            assert(grid.width() == width);
            assert(grid.height() == height);

            sf::Image gridImage;
            gridImage.create(width, height);

            for (size_t x = 0; x < grid.width(); x++)
            {
                for (size_t y = 0; y < grid.height(); y++)
                {
                    uint8_t c = (uint8_t)(grid.get(x, y) * 255);
                    sf::Color color(c, c, c);
                    gridImage.setPixel(x, y, color); 
                }
            }

            m_gridImages.push_back(gridImage);
        }

        m_contourImage.create(width, height);

        m_occupancyImage.create(width, height);

        m_backgroundTexture.loadFromImage(m_occupancyImage);
        m_backgroundSprite.setTexture(m_backgroundTexture);

        // Make grid image 0 the default
        //m_backgroundImagePtr = NULL;
        m_backgroundImagePtr = &m_gridImages[0];
    }

    void rotateRobots(double angle)
    {
        for (auto & entity : m_sim->getWorld()->getEntities("robot"))
        { 
            if (!entity.hasComponent<CSteer>()) { continue; }
            auto & steer     = entity.getComponent<CSteer>();
            steer.angle += angle;
        }        
    }
        
    void sUserInput()
    {
        sf::Event event;
        while (m_window.pollEvent(event))
        {
            // this event triggers when the window is closed
            if (event.type == sf::Event::Closed)
            {
                exit(0);
            }

            // this event is triggered when a key is pressed
            if (event.type == sf::Event::KeyPressed)
            {
                switch (event.key.code)
                {
                    case sf::Keyboard::Escape: exit(0); break;
                    case sf::Keyboard::D:      m_debug = !m_debug; break;
                    case sf::Keyboard::S:      m_sensors = !m_sensors; break;
                    case sf::Keyboard::C:
                        m_backgroundImagePtr = &m_contourImage;
                        break;
                    case sf::Keyboard::O:
                        m_backgroundImagePtr = &m_occupancyImage;
                        break;
                    case sf::Keyboard::R:
                        m_backgroundImagePtr = &m_gridImages[0];
                        break;
                    case sf::Keyboard::G:
                        m_backgroundImagePtr = &m_gridImages[1];
                        break;
                    case sf::Keyboard::Num0:
                        m_backgroundImagePtr = NULL;
                        break;
                    case sf::Keyboard::Left:
                        rotateRobots(-0.1);
                        break;
                    case sf::Keyboard::Right:
                        rotateRobots(0.1);
                        break;
                    case sf::Keyboard::Up:
                        m_upArrowCallback();
                        break;
                    case sf::Keyboard::Down:
                        m_downArrowCallback();
                        break;
                    case sf::Keyboard::Space:
                        m_spaceCallback();
                        break;

                    default: break;
                }
            }

            if (event.type == sf::Event::MouseButtonPressed)
            {
                if (event.mouseButton.button == sf::Mouse::Left)
                {
                    m_leftMouseDown = true;
                }

                if (event.mouseButton.button == sf::Mouse::Right)
                {
                    for (auto e : m_sim->getWorld()->getEntities())
                    {
                        Vec2 mPos((double)event.mouseButton.x, (double)event.mouseButton.y);
                        if (mPos.dist(e.getComponent<CTransform>().p) < e.getComponent<CCircleBody>().r)
                        {
                            // Toggle selection.
                            if (m_selected && m_selected.id() == e.id())
                                m_selected = Entity();
                            else
                                m_selected = e;
                            break;
                        }
                    }

                }

                /*
                if (event.mouseButton.button == sf::Mouse::Right)
                {
                    for (auto & e : m_sim->getWorld()->getEntities())
                    {
                        Vec2 mPos((double)event.mouseButton.x, (double)event.mouseButton.y);
                        if (mPos.dist(e.getComponent<CTransform>().p) < e.getComponent<CCircleBody>().r)
                        {
                            if (e.hasComponent<CSteer>()) {
                                auto & steer = e.getComponent<CSteer>();
                                steer.frozen = !steer.frozen;
                            }
                            break;
                        }
                    }
                }
                */
            }

            if (event.type == sf::Event::MouseButtonReleased)
            {
                if (event.mouseButton.button == sf::Mouse::Left)
                {
                    m_leftMouseDown = false;
                }

            }

            if (event.type == sf::Event::MouseMoved && m_leftMouseDown)
            {
                m_mousePos = sf::Vector2f((float)event.mouseMove.x, (float)event.mouseMove.y);

                for (auto e : m_sim->getWorld()->getEntities())
                {
                    Vec2 mPos((double)event.mouseMove.x, (double)event.mouseMove.y);
                    if (mPos.dist(e.getComponent<CTransform>().p) < e.getComponent<CCircleBody>().r)
                    {
                        auto & t = e.getComponent<CTransform>();
                        Vec2 diff(m_mousePos.x - t.p.x, m_mousePos.y - t.p.y);
                        t.p += diff;
                    }
                }

            }
        }

    }


    void drawLine(Vec2 p1, Vec2 p2, sf::Color color)
    {
        sf::Vertex line[] =
        {
            sf::Vertex(sf::Vector2f((float)p1.x, (float)p1.y), color),
            sf::Vertex(sf::Vector2f((float)p2.x, (float)p2.y), color)
        };

        m_window.draw(line, 2, sf::Lines);
    }

    void sRender()
    {
        m_window.clear();

        // Fill the occupancy grid
        sf::Color color(255, 255, 255);
        for (auto robot : m_sim->getWorld()->getEntities("robot"))
        {
            auto & t = robot.getComponent<CTransform>();
            m_occupancyImage.setPixel((int)t.p.x, (int)t.p.y, color);   
        }


        // Draw the background image
        if (m_backgroundImagePtr != NULL)
        {
            m_backgroundTexture.update(*m_backgroundImagePtr);
            m_window.draw(m_backgroundSprite);
        }

 
        // draw robot plows
        for (auto e : m_sim->getWorld()->getEntities())
        {
            if (!e.hasComponent<CPlowBody>()) { continue; }

            auto & t = e.getComponent<CTransform>();
            auto & pb = e.getComponent<CPlowBody>();
            auto & c = e.getComponent<CColor>();
            auto & steer = e.getComponent<CSteer>();
    
            pb.shape.setPosition((float)t.p.x, (float)t.p.y);
            pb.shape.setRotation(steer.angle * 180.0 / M_PI);
            if (steer.slowedCount > 0) {
                pb.shape.setFillColor(sf::Color(255, 0, 0));                
            } else {
                pb.shape.setFillColor(sf::Color(c.r, c.g, c.b));                
            }
            m_window.draw(pb.shape);

            // Draw a line along the prow.
            Vec2 prow(t.p.x + pb.length * cos(steer.angle),
                      t.p.y + pb.length * sin(steer.angle));
            drawLine(t.p, prow, sf::Color(255, 255, 255));
        }

        // draw robot spokes
        for (auto e : m_sim->getWorld()->getEntities())
        {
            if (!e.hasComponent<CSpoke>()) { continue; }

            auto & t = e.getComponent<CTransform>();
            auto & spoke = e.getComponent<CSpoke>();
            auto & c = e.getComponent<CColor>();
            auto & steer = e.getComponent<CSteer>();
    
            double x0 = t.p.x + spoke.innerRadius * cos(steer.angle + spoke.innerAngle);
            double y0 = t.p.y + spoke.innerRadius * sin(steer.angle + spoke.innerAngle);
            double x1 = t.p.x + spoke.outerRadius * cos(steer.angle + spoke.outerAngle);
            double y1 = t.p.y + spoke.outerRadius * sin(steer.angle + spoke.outerAngle);

            drawLine(Vec2(x0, y0), Vec2(x1, y1), sf::Color(255, 255, 255));
        }

        // draw circles
        for (auto e : m_sim->getWorld()->getEntities())
        {
            if (!e.hasComponent<CCircleShape>()) { continue; }

            auto & t = e.getComponent<CTransform>();
            auto & s = e.getComponent<CCircleShape>();
            auto & c = e.getComponent<CColor>();

            s.shape.setPosition((float)t.p.x, (float)t.p.y);
            s.shape.setFillColor(sf::Color(c.r, c.g, c.b, c.a));

            if (e.hasComponent<CSteer>()) {
                auto & steer = e.getComponent<CSteer>();
                if (steer.frozen)
                    s.shape.setFillColor(sf::Color(127, 127, 127, 127));
            }

            m_window.draw(s.shape);

            Vec2 velPoint;
            double vLength = t.v.length();
            if (vLength == 0)
            {
                velPoint = Vec2(t.p.x + s.shape.getRadius(), t.p.y);
                continue;
            }
            else
            {
                velPoint = t.p + t.v.normalize() * s.shape.getRadius();
            }

            drawLine(t.p, velPoint, sf::Color(255, 255, 255));
        }

        // draw robot sensors
        if (m_sensors)
        {
            float gridSensorRadius = 2;

            for (auto robot : m_sim->getWorld()->getEntities("robot"))
            {
                if (!m_selected || robot.id() != m_selected.id()) { continue; }

                if (!robot.hasComponent<CSensorArray>()) { continue; }
                auto & sensors = robot.getComponent<CSensorArray>();
                auto & c = robot.getComponent<CColor>();

                // Position and angle of robot.
                const Vec2 & pos = robot.getComponent<CTransform>().p;
                double theta = robot.getComponent<CSteer>().angle;

                sf::Color grey(127, 127, 127);
                sf::Color white(255, 255, 255);
                for (auto sensor : sensors.cameraSensors)
                {
                    std::vector<bool> reading = sensor->getReading(m_sim->getWorld());
                    for (int i = 0; i<reading.size(); i++) {
                        Vec2 p1, p2;
                        sensor->getSegmentStartEnd(i, p1, p2);
                        if (reading[i])
                            drawLine(p1, p2, white);
                        else
                            drawLine(p1, p2, grey);
                    }
                }

                for (auto sensor : sensors.fancySensors)
                {
                    double reading = sensor->getReading(m_sim->getWorld());
                    for (int i=0; i<sensor->getNumberOfCircles(); i++) {

                        sf::CircleShape cShape((float)sensor->getCircleRadius(i), 132);
                        if (sensor->m_typeName == "red_puck")
                            cShape.setFillColor(sf::Color(255, 0, 0, 80)); 
                        if (sensor->m_typeName == "robot")
                            cShape.setFillColor(sf::Color(0, 0, 255, 80)); 

                        cShape.setOrigin((float)sensor->getCircleRadius(i), (float)sensor->getCircleRadius(i));
                        Vec2 pos = sensor->getCirclePosition(i);
                        cShape.setPosition((float)pos.x, (float)pos.y);

                        if (reading > 0) { 
                            cShape.setOutlineThickness(1);
                        } else { 
                            cShape.setOutlineThickness(0);
                        }
                        m_window.draw(cShape);
                    }
                }

                for (auto & sensor : sensors.gridSensors)
                {
                    sf::CircleShape sensorShape(gridSensorRadius, 32);
                    sensorShape.setOrigin(gridSensorRadius, gridSensorRadius);
                    Vec2 pos = sensor->getPosition();
                    sensorShape.setPosition((float)pos.x, (float)pos.y);
                    sensorShape.setFillColor(sf::Color::White);
                    m_window.draw(sensorShape);
                }

                for (auto sensor : sensors.obstacleSensors)
                {
                    sf::CircleShape sensorShape((float)sensor->radius(), 32);
                    sensorShape.setOrigin((float)sensor->radius(), (float)sensor->radius());
                    Vec2 pos = sensor->getPosition();
                    sensorShape.setPosition((float)pos.x, (float)pos.y);
                    double reading = sensor->getReading(m_sim->getWorld());
                    if (reading > 0) { sensorShape.setFillColor(sf::Color(255, 255, 255, 80)); }
                    else { sensorShape.setFillColor(sf::Color(0, 255, 255, 80)); }
                    m_window.draw(sensorShape);
                }

                for (auto sensor : sensors.robotSensors)
                {
                    sf::CircleShape sensorShape((float)sensor->radius(), 32);
                    sensorShape.setOrigin((float)sensor->radius(), (float)sensor->radius());
                    Vec2 pos = sensor->getPosition();
                    sensorShape.setPosition((float)pos.x, (float)pos.y);
                    double reading = sensor->getReading(m_sim->getWorld());
                    if (reading > 0) { sensorShape.setFillColor(sf::Color(255, 255, 255, 80)); }
                    else { sensorShape.setFillColor(sf::Color(255, 255, 0, 80)); }
                    m_window.draw(sensorShape);
                }

                for (auto sensor : sensors.puckSensors)
                {
                    sf::CircleShape sensorShape((float)sensor->radius(), 132);
                    sensorShape.setOrigin((float)sensor->radius(), (float)sensor->radius());
                    Vec2 pos = sensor->getPosition();
                    sensorShape.setPosition((float)pos.x, (float)pos.y);
                    double reading = sensor->getReading(m_sim->getWorld());
                    if (reading > 0) { sensorShape.setFillColor(sf::Color(255, 255, 255, 80)); }
                    else { sensorShape.setFillColor(sf::Color(c.r, c.g, c.b, 80)); }
                    m_window.draw(sensorShape);
                }
            }
        }

        // Draw other robot-specific "decorations".
        for (auto robot : m_sim->getWorld()->getEntities("robot"))
        {
            auto & t = robot.getComponent<CTransform>();
            auto & s = robot.getComponent<CCircleShape>();
            auto & c = robot.getComponent<CColor>();

            if (!robot.hasComponent<CVectorIndicator>()) { continue; }

            auto & steer = robot.getComponent<CSteer>();
            auto & vi = robot.getComponent<CVectorIndicator>();

            Vec2 start(t.p.x, t.p.y);
            Vec2 end(t.p.x + vi.length * cos(steer.angle + vi.angle),
                     t.p.y + vi.length * sin(steer.angle + vi.angle));
            drawLine(start, end, sf::Color(vi.r, vi.g, vi.b, vi.a));
        }

        for (auto & e : m_sim->getWorld()->getEntities("line"))
        {
            auto & line = e.getComponent<CLineBody>();

            sf::CircleShape circle((float)line.r, 32);
            circle.setOrigin((float)line.r, (float)line.r);
            circle.setOutlineThickness(1);
            circle.setPosition((float)line.s.x, (float)line.s.y);
            m_window.draw(circle);
            circle.setPosition((float)line.e.x, (float)line.e.y);
            m_window.draw(circle);

            Vec2 normal(-(line.e.y - line.s.y), (line.e.x - line.s.x));
            normal = normal.normalize() * line.r;

            drawLine(line.s + normal, line.e + normal, sf::Color::White);
            drawLine(line.s - normal, line.e - normal, sf::Color::White);
        }

        if (m_debug)
        {
            for (auto & collision : m_sim->getCollisions())
            {
                drawLine(collision.t1->p, collision.t2->p, sf::Color::Green);
            }
        }

        if (m_selected != Entity() && m_selected.hasComponent<CSensorArray>())
        {
            auto & t = m_selected.getComponent<CTransform>();
            SensorReading reading;
            SensorTools::ReadSensorArray(m_selected, m_sim->getWorld(), reading);

            sf::Text text;
            text.setFont(m_font);
            text.setCharacterSize(12);
            text.setString(reading.toString());
            //text.setPosition((float)t.p.x, (float)t.p.y);
            float w = m_sim->getWorld()->width();
            float h = m_sim->getWorld()->height();
            float textX = w / 2.0f;
            float textY = h / 2.0f;
            text.setPosition(textX, textY);
            text.setFillColor(sf::Color::White);

            // Get a rectangle to draw in the background, making the text easier to see.
            sf::FloatRect backgroundRect = text.getLocalBounds();
            sf::RectangleShape background(sf::Vector2f(backgroundRect.width, backgroundRect.height));
            background.setFillColor(sf::Color::Black);
            sf::Transform xform = text.getTransform();
            // TRYING TO RESCALE WINDOW BUT ITS NOT WORKING
            //m_window.draw(background, xform.scale(1.5f, 1.5f, textX, textY));
            m_window.draw(background, xform);
        
            m_window.draw(text);
        }
        
        // draw information
        /*
        std::stringstream ss;
        ss << "Num Objs: " << m_sim->getWorld()->getEntities().size() << "\n";
        ss << "CPU Time: " << m_sim->getComputeTime() << "ms\n";
        ss << "Max Time: " << m_sim->getComputeTimeMax() << "ms\n";
        m_text.setString(ss.str());
        m_window.draw(m_text);
        */

        // draw evaluation
        sf::Text text;
        text.setFont(m_font);
        text.setString(m_status);
        text.setCharacterSize(20);
        text.setPosition(5, (float)m_sim->getWorld()->height() - text.getLocalBounds().height-5);

        // Now draw the text itself.
        m_window.draw(text);

        m_window.display();
    }
    
public:

    GUI(std::shared_ptr<Simulator> sim, size_t fps)
        : m_sim(sim)
    {
        m_window.create(sf::VideoMode((size_t)m_sim->getWorld()->width(), (size_t)m_sim->getWorld()->height()), "CWaggle");
        m_window.setFramerateLimit(fps);

        // Scale the window size up for high-res screens.
        int scaleFactor = 1;
        m_window.setSize(sf::Vector2u(scaleFactor*m_sim->getWorld()->width(), scaleFactor*m_sim->getWorld()->height()));
        init(sim);
    }

    void setStatus(const std::string & str)
    {
        m_status = str;
    }

    void setSim(std::shared_ptr<Simulator> sim)
    {
        init(sim);
    }

    void update()
    {
        sUserInput();
        sRender();
    }

    void close()
    {
        m_window.close();
    }

    void addContour(double contourValue) {
        // If there is no grid, then there can be no contour.
        auto & grid = m_sim->getWorld()->getGrid(0);
        if (grid.width() == 0)
            return;

        sf::Color white(255, 255, 255);
        for (size_t x = 1; x < grid.width()-1; x++)
        {
            for (size_t y = 1; y < grid.height()-1; y++)
            {
                // Compute difference between current cell and given value.
                double diff = fabs(grid.get(x, y) - contourValue);

                if (diff > 0.1)
                    continue;

                // Count how many neighbour cells have a smaller difference.
                int smallerCount = 0;
                for (int dx = -10; dx <= 10; dx++) {
                    for (int dy = -10; dy <= 10; dy++) {
                        if (!(dx == 0 && dy == 0) &&
                            fabs(grid.get(x+dx, y+dy) - contourValue) <= diff) {
                            smallerCount++;
                        }
                    }
                }
//                std::cout << diff << ", " << smallerCount << std::endl;

                if (smallerCount <= 85) {
                //    sf::Color white((int)(255.0 * diff), (int)(255.0 * diff), (int)(255.0 * diff));
                    m_contourImage.setPixel(x, y, white); 
                }
            }
        }
    } 

    void updateGridImage(int gridIndex) {
        auto & grid = m_sim->getWorld()->getGrid(gridIndex);

        int w = grid.width();
        int h = grid.height();

        auto & gridImage = m_gridImages[gridIndex];

        for (size_t x = 0; x < w; x++) {
            for (size_t y = 0; y < h; y++) {
                uint8_t c = (uint8_t)(grid.get(x, y) * 255);
                sf::Color color(c, c, c);
                gridImage.setPixel(x, y, color); 
            }
        }
    }

    void setUpArrowCallback(callback_function pFunc) {
        m_upArrowCallback = pFunc;
    }   
    void setDownArrowCallback(callback_function pFunc) {
        m_downArrowCallback = pFunc;
    }   
    void setSpaceCallback(callback_function pFunc) {
        m_spaceCallback = pFunc;
    }   

    void saveScreenshot(std::string filename) {
        //sf::Image screenshot = m_window.capture();
        //screenshot.saveToFile(filename);
        sf::Texture texture;
        texture.create(m_window.getSize().x, m_window.getSize().y);
        texture.update(m_window);
        texture.copyToImage().saveToFile(filename);
    }
};
