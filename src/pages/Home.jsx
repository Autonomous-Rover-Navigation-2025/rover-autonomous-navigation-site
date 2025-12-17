import { Link } from "react-router-dom";
import styled from "styled-components";
import { HeroAnimated } from "../components/Hero";

/* ---------- Layout ---------- */
const Page = styled.section`
  display: grid;
  gap: 56px;
`;

const Section = styled.section``;

const Divider = styled.hr`
  border: 0;
  border-top: 1px solid #e2e8f0;
  margin: 0;
`;

/* ---------- Section headers ---------- */
const SectionHeader = styled.h2`
  margin: 0 0 14px;
  font-size: 14px;
  font-weight: 700;
  letter-spacing: 0.08em;
  text-transform: uppercase;
  color: #64748b;
`;

/* ---------- Two-column (News / Publications) ---------- */
const TwoCol = styled.div`
  display: grid;
  gap: 24px;
  grid-template-columns: 1fr;
  @media (min-width: 980px) {
    grid-template-columns: 1.1fr 1fr;
  }
`;

const List = styled.ul`
  margin: 0;
  padding: 0;
  list-style: none;
  display: grid;
  gap: 10px;
`;

const MembersList = styled.ul`
  margin: 0;
  padding: 0;
  list-style: none;
  display: grid;
  gap: 10px;
  grid-template-columns: 1fr;
  @media (min-width: 720px) {
    grid-template-columns: 1fr 1fr;
  }
`;

const Item = styled.li`
  border: 1px solid #e2e8f0;
  border-radius: 12px;
  padding: 12px;
  background: #fff;
  display: grid;
  gap: 6px;
`;

const MemberInfo = styled.div`
  display: flex;
  flex-direction: column;
  gap: 4px;
`;

const MemberName = styled.strong`
  font-size: 15px;
  color: #0f172a;
`;

const MemberRole = styled.span`
  font-size: 13.5px;
  color: #475569;
`;

const LinkedInLink = styled.a`
  display: inline-flex;
  align-items: center;
  gap: 6px;
  margin-top: 4px;
  color: #0077b5;
  text-decoration: none;
  font-size: 13px;
  transition: color 0.15s ease;
  &:hover {
    color: #005885;
  }
  &:focus-visible {
    outline: 2px solid #0077b5;
    outline-offset: 2px;
    border-radius: 4px;
  }
`;

const LinkedInIcon = styled.svg`
  width: 16px;
  height: 16px;
  fill: currentColor;
  flex-shrink: 0;
`;

const Meta = styled.span`
  font-size: 12px;
  color: #64748b;
`;

const Linkish = styled(Link)`
  text-decoration: none;
  color: #0f172a;
  &:hover {
    text-decoration: underline;
  }
`;

/* ---------- Pretty stack cards (hardware + software) ---------- */
const StackGrid = styled.div`
  display: grid;
  gap: 16px;
  grid-template-columns: 1fr;
  @media (min-width: 720px) {
    grid-template-columns: 1fr 1fr;
  }
  @media (min-width: 1024px) {
    grid-template-columns: 1fr 1fr 1fr;
  }
`;

const CardPretty = styled.article`
  border: 1px solid #e2e8f0;
  border-radius: 16px;
  padding: 14px;
  background: #fff;
  box-shadow: 0 1px 0 rgba(2, 6, 23, 0.04);
  transition: transform 0.12s ease, box-shadow 0.12s ease,
    border-color 0.12s ease;
  &:hover {
    transform: translateY(-2px);
    box-shadow: 0 6px 16px rgba(2, 6, 23, 0.08);
    border-color: #dbe2ea;
  }
`;

const Thumb = styled.div`
  /* Rectangular frame */
  aspect-ratio: 16 / 9; /* or: height: 180px; */
  padding: 12px;
  border: 1px solid #e2e8f0;
  border-radius: 12px;

  /* Center content */
  display: flex;
  align-items: center;
  justify-content: center;

  overflow: hidden; /* safe, nothing will be clipped with max-* */
`;

const ThumbImg = styled.img`
  /* Important: don't force width/height to 100% */
  max-width: 100%;
  max-height: 100%;
  width: auto !important;
  height: auto !important;
  object-fit: contain !important;
  object-position: center;
  display: block;
`;

const CardKicker = styled.div`
  font-size: 11px;
  letter-spacing: 0.08em;
  color: #64748b;
  text-transform: uppercase;
  margin-top: 4px;
`;

const CardTitle = styled.h3`
  margin: 6px 0 6px;
  font-size: 16px;
  font-weight: 700;
`;

const CardBodySm = styled.p`
  margin: 0;
  font-size: 13.5px;
  color: #475569;
  line-height: 1.55;
`;

const Pills = styled.div`
  display: flex;
  flex-wrap: wrap;
  gap: 6px;
  margin-top: 10px;
`;
const Pill = styled.span`
  display: inline-block;
  padding: 4px 8px;
  border-radius: 999px;
  font-size: 12px;
  background: #f1f5f9;
  color: #0f172a;
  border: 1px solid #e2e8f0;
`;

/* ---------- Demo Video Section ---------- */
const VideoContainer = styled.div`
  width: 100%;
  max-width: 1100px;
  margin: 0 auto;
`;

const VideoWrapper = styled.div`
  position: relative;
  width: 100%;
  aspect-ratio: 16 / 9;
  border-radius: 16px;
  overflow: hidden;
  border: 1px solid #e2e8f0;
  box-shadow: 0 4px 12px rgba(2, 6, 23, 0.08);
  background: #000;
`;

const VideoIframe = styled.iframe`
  width: 100%;
  height: 100%;
  border: none;
  display: block;
`;

const VideoCaption = styled.p`
  margin: 16px 0 0;
  text-align: center;
  font-size: 14px;
  color: #64748b;
  line-height: 1.6;
`;

export default function Home() {
  return (
    <Page>
      <HeroAnimated />

      {/* DEMO VIDEO SECTION */}
      <Section>
        <SectionHeader>Demo Video</SectionHeader>
        <VideoContainer>
          <VideoWrapper>
            <VideoIframe
              src="https://www.youtube.com/embed/9r4AT93h5W8"
              title="Rover Autonomous Navigation Demo"
              allow="accelerometer; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
              allowFullScreen
            />
          </VideoWrapper>
          <VideoCaption>
            Watch the rover navigate autonomously indoor and outdoor, both in
            daytime and nighttime.
          </VideoCaption>
        </VideoContainer>
      </Section>

      <Divider />

      <Section>
        <SectionHeader>Hardware</SectionHeader>
        <StackGrid>
          {/* Example hardware card with image (put files in /public/hardware/...) */}
          <CardPretty>
            <Thumb>
              <ThumbImg
                src="/hardware/jetson.jpg"
                alt="NVIDIA Jetson Xavier NX"
              />
            </Thumb>
            <CardKicker>Compute</CardKicker>
            <CardTitle>NVIDIA Jetson Xavier NX</CardTitle>
            <CardBodySm>
              On-board compute for real-time perception and planning; compact,
              power-efficient module designed for field robots.
            </CardBodySm>
            <Pills>
              <Pill>CUDA</Pill>
              <Pill>Edge AI</Pill>
              <Pill>ROS 2</Pill>
            </Pills>
          </CardPretty>

          <CardPretty>
            <Thumb>
              <ThumbImg src="/hardware/rplidar.png" alt="RPLIDAR A2M7" />
            </Thumb>
            <CardKicker>Sensing</CardKicker>
            <CardTitle>360° LiDAR (RPLIDAR A2M7)</CardTitle>
            <CardBodySm>
              Planar LiDAR for reliable 2D mapping and near-field obstacle
              checks in hallways and open paths.
            </CardBodySm>
            <Pills>
              <Pill>2D Scan</Pill>
              <Pill>10–12 Hz</Pill>
              <Pill>360°</Pill>
            </Pills>
          </CardPretty>

          <CardPretty>
            <Thumb>
              <ThumbImg src="/hardware/camera.jpg" alt="Depth Camera" />
            </Thumb>
            <CardKicker>Sensing</CardKicker>
            <CardTitle>Depth Camera</CardTitle>
            <CardBodySm>
              Dense depth for 3D scene understanding and obstacle perception;
              complements LiDAR for richer geometry.
            </CardBodySm>
            <Pills>
              <Pill>RGB-D</Pill>
              <Pill>3D</Pill>
              <Pill>Point Cloud</Pill>
            </Pills>
          </CardPretty>

          <CardPretty>
            <Thumb>
              <ThumbImg src="/hardware/sabertooth.jpeg" alt="IMU" />
            </Thumb>
            <CardKicker>Control</CardKicker>
            <CardTitle>Sabertooth</CardTitle>
            <CardBodySm>
              Dual-channel DC motor driver (12A continuous, 25A peak) for quick
              acceleration and reliable braking.
            </CardBodySm>
            <Pills>
              <Pill>UART</Pill>
              <Pill>Dual 12A motors</Pill>
            </Pills>
          </CardPretty>

          <CardPretty>
            <Thumb>
              <ThumbImg src="/hardware/imu.jpg" alt="IMU" />
            </Thumb>
            <CardKicker>State Estimation</CardKicker>
            <CardTitle>IMU</CardTitle>
            <CardBodySm>
              Inertial sensing improves short-term heading and motion estimates,
              stabilizing localization in low-texture scenes.
            </CardBodySm>
            <Pills>
              <Pill>Gyro</Pill>
              <Pill>Accel</Pill>
            </Pills>
          </CardPretty>

          <CardPretty>
            <Thumb>
              <ThumbImg
                src="/hardware/wheel-encoder.jpeg"
                alt="Wheel Encoders"
              />
            </Thumb>
            <CardKicker>State Estimation</CardKicker>
            <CardTitle>Wheel Encoders</CardTitle>
            <CardBodySm>
              Odometry from encoders anchors the motion model and helps bridge
              gaps between visual and LiDAR updates.
            </CardBodySm>
            <Pills>
              <Pill>Odometry</Pill>
              <Pill>Low-drift short-term</Pill>
            </Pills>
          </CardPretty>
        </StackGrid>
      </Section>

      <Divider />

      {/* SOFTWARE STACK */}
      <Section>
        <SectionHeader>Software</SectionHeader>
        <StackGrid>
          <CardPretty>
            <CardKicker>Robotics</CardKicker>
            <CardTitle>NVIDIA Isaac ROS</CardTitle>
            <CardBodySm>
              GPU-accelerated ROS 2 components that speed image/point-cloud
              pipelines and cut CPU load.
            </CardBodySm>
            <Pills>
              <Pill>GPU</Pill>
              <Pill>ROS 2</Pill>
              <Pill>Isaac ROS</Pill>
            </Pills>
          </CardPretty>

          <CardPretty>
            <CardKicker>Navigation</CardKicker>
            <CardTitle>LiDAR-Based 2D Navigation</CardTitle>
            <CardBodySm>
              2D mapping and navigation with LiDAR, SLAM Toolbox, and AMCL;
              camera cues refine object awareness.
            </CardBodySm>
            <Pills>
              <Pill>LiDAR</Pill>
              <Pill>SLAM Toolbox</Pill>
              <Pill>AMCL</Pill>
            </Pills>
          </CardPretty>

          <CardPretty>
            <CardKicker>Navigation</CardKicker>
            <CardTitle>Vision-Based 3D Navigation</CardTitle>
            <CardBodySm>
              Visual SLAM feeds NVBlox to build local 3D maps; heavier
              point-cloud work runs on the server.
            </CardBodySm>
            <Pills>
              <Pill>VSLAM</Pill>
              <Pill>NVBlox</Pill>
              <Pill>RGB-D</Pill>
            </Pills>
          </CardPretty>

          <CardPretty>
            <CardKicker>Control</CardKicker>
            <CardTitle>State Estimation & Motion Control</CardTitle>
            <CardBodySm>
              Wheel-encoder odometry fused with IMU via EKF for stable pose;
              commands drive Sabertooth for smooth tracking.
            </CardBodySm>
            <Pills>
              <Pill>EKF</Pill>
              <Pill>IMU</Pill>
              <Pill>Sabertooth</Pill>
            </Pills>
          </CardPretty>

          <CardPretty>
            <CardKicker>AI/ML</CardKicker>
            <CardTitle>Conversational Voice (LLM)</CardTitle>
            <CardBodySm>
              Hands-free control with a wake word, on-device ASR, and a
              lightweight LLM for natural, reliable commands.
            </CardBodySm>
            <Pills>
              <Pill>Edge AI</Pill>
              <Pill>ASR</Pill>
              <Pill>LLM</Pill>
            </Pills>
          </CardPretty>

          <CardPretty>
            <CardKicker>DevOps</CardKicker>
            <CardTitle>Containerized Runtime (Docker)</CardTitle>
            <CardBodySm>
              Services run in Docker for reproducible builds and quick profile
              swaps; Jetson-optimized, versioned deployments.
            </CardBodySm>
            <Pills>
              <Pill>Docker</Pill>
              <Pill>CI/CD</Pill>
            </Pills>
          </CardPretty>
        </StackGrid>
      </Section>

      <Divider />

      {/* NEWS + PUBLICATIONS */}
      <Section>
        {/* MEMBERS */}
        <SectionHeader>Team Members</SectionHeader>
        <MembersList>
          <Item>
            <MemberInfo>
              <MemberName>Marco Levorato</MemberName>
              <MemberRole>Professor / Project Advisor</MemberRole>
              <LinkedInLink
                href="https://www.linkedin.com/in/marco-levorato-424a967/"
                target="_blank"
                rel="noopener noreferrer"
              >
                <LinkedInIcon
                  viewBox="0 0 24 24"
                  xmlns="http://www.w3.org/2000/svg"
                >
                  <path d="M20.447 20.452h-3.554v-5.569c0-1.328-.027-3.037-1.852-3.037-1.853 0-2.136 1.445-2.136 2.939v5.667H9.351V9h3.414v1.561h.046c.477-.9 1.637-1.85 3.37-1.85 3.601 0 4.267 2.37 4.267 5.455v6.286zM5.337 7.433c-1.144 0-2.063-.926-2.063-2.065 0-1.138.92-2.063 2.063-2.063 1.14 0 2.064.925 2.064 2.063 0 1.139-.925 2.065-2.064 2.065zm1.782 13.019H3.555V9h3.564v11.452zM22.225 0H1.771C.792 0 0 .774 0 1.729v20.542C0 23.227.792 24 1.771 24h20.451C23.2 24 24 23.227 24 22.271V1.729C24 .774 23.2 0 22.222 0h.003z" />
                </LinkedInIcon>
                LinkedIn
              </LinkedInLink>
            </MemberInfo>
          </Item>
          <Item>
            <MemberInfo>
              <MemberName>Chaerin An</MemberName>
              <MemberRole>Embedded Software Engineer</MemberRole>
              <LinkedInLink
                href="https://www.linkedin.com/in/chaerin00/"
                target="_blank"
                rel="noopener noreferrer"
              >
                <LinkedInIcon
                  viewBox="0 0 24 24"
                  xmlns="http://www.w3.org/2000/svg"
                >
                  <path d="M20.447 20.452h-3.554v-5.569c0-1.328-.027-3.037-1.852-3.037-1.853 0-2.136 1.445-2.136 2.939v5.667H9.351V9h3.414v1.561h.046c.477-.9 1.637-1.85 3.37-1.85 3.601 0 4.267 2.37 4.267 5.455v6.286zM5.337 7.433c-1.144 0-2.063-.926-2.063-2.065 0-1.138.92-2.063 2.063-2.063 1.14 0 2.064.925 2.064 2.063 0 1.139-.925 2.065-2.064 2.065zm1.782 13.019H3.555V9h3.564v11.452zM22.225 0H1.771C.792 0 0 .774 0 1.729v20.542C0 23.227.792 24 1.771 24h20.451C23.2 24 24 23.227 24 22.271V1.729C24 .774 23.2 0 22.222 0h.003z" />
                </LinkedInIcon>
                LinkedIn
              </LinkedInLink>
            </MemberInfo>
          </Item>
          <Item>
            <MemberInfo>
              <MemberName>Selina Shrestha</MemberName>
              <MemberRole>Software and Machine Learning Engineer</MemberRole>
              <LinkedInLink
                href="https://www.linkedin.com/in/selinashrestha/"
                target="_blank"
                rel="noopener noreferrer"
              >
                <LinkedInIcon
                  viewBox="0 0 24 24"
                  xmlns="http://www.w3.org/2000/svg"
                >
                  <path d="M20.447 20.452h-3.554v-5.569c0-1.328-.027-3.037-1.852-3.037-1.853 0-2.136 1.445-2.136 2.939v5.667H9.351V9h3.414v1.561h.046c.477-.9 1.637-1.85 3.37-1.85 3.601 0 4.267 2.37 4.267 5.455v6.286zM5.337 7.433c-1.144 0-2.063-.926-2.063-2.065 0-1.138.92-2.063 2.063-2.063 1.14 0 2.064.925 2.064 2.063 0 1.139-.925 2.065-2.064 2.065zm1.782 13.019H3.555V9h3.564v11.452zM22.225 0H1.771C.792 0 0 .774 0 1.729v20.542C0 23.227.792 24 1.771 24h20.451C23.2 24 24 23.227 24 22.271V1.729C24 .774 23.2 0 22.222 0h.003z" />
                </LinkedInIcon>
                LinkedIn
              </LinkedInLink>
            </MemberInfo>
          </Item>
          <Item>
            <MemberInfo>
              <MemberName>Farhin Salman Mansuri </MemberName>
              <MemberRole>Hardware and Embedded Software Developer</MemberRole>
              <LinkedInLink
                href="https://www.linkedin.com/in/farhin01/"
                target="_blank"
                rel="noopener noreferrer"
              >
                <LinkedInIcon
                  viewBox="0 0 24 24"
                  xmlns="http://www.w3.org/2000/svg"
                >
                  <path d="M20.447 20.452h-3.554v-5.569c0-1.328-.027-3.037-1.852-3.037-1.853 0-2.136 1.445-2.136 2.939v5.667H9.351V9h3.414v1.561h.046c.477-.9 1.637-1.85 3.37-1.85 3.601 0 4.267 2.37 4.267 5.455v6.286zM5.337 7.433c-1.144 0-2.063-.926-2.063-2.065 0-1.138.92-2.063 2.063-2.063 1.14 0 2.064.925 2.064 2.063 0 1.139-.925 2.065-2.064 2.065zm1.782 13.019H3.555V9h3.564v11.452zM22.225 0H1.771C.792 0 0 .774 0 1.729v20.542C0 23.227.792 24 1.771 24h20.451C23.2 24 24 23.227 24 22.271V1.729C24 .774 23.2 0 22.222 0h.003z" />
                </LinkedInIcon>
                LinkedIn
              </LinkedInLink>
            </MemberInfo>
          </Item>
          <Item>
            <MemberInfo>
              <MemberName>Sahil Mukeshbhai Kakadiya</MemberName>
              <MemberRole>
                IOT Communication and Embedded Software Developer
              </MemberRole>
              <LinkedInLink
                href="https://www.linkedin.com/in/sahil-kakadiya/"
                target="_blank"
                rel="noopener noreferrer"
              >
                <LinkedInIcon
                  viewBox="0 0 24 24"
                  xmlns="http://www.w3.org/2000/svg"
                >
                  <path d="M20.447 20.452h-3.554v-5.569c0-1.328-.027-3.037-1.852-3.037-1.853 0-2.136 1.445-2.136 2.939v5.667H9.351V9h3.414v1.561h.046c.477-.9 1.637-1.85 3.37-1.85 3.601 0 4.267 2.37 4.267 5.455v6.286zM5.337 7.433c-1.144 0-2.063-.926-2.063-2.065 0-1.138.92-2.063 2.063-2.063 1.14 0 2.064.925 2.064 2.063 0 1.139-.925 2.065-2.064 2.065zm1.782 13.019H3.555V9h3.564v11.452zM22.225 0H1.771C.792 0 0 .774 0 1.729v20.542C0 23.227.792 24 1.771 24h20.451C23.2 24 24 23.227 24 22.271V1.729C24 .774 23.2 0 22.222 0h.003z" />
                </LinkedInIcon>
                LinkedIn
              </LinkedInLink>
            </MemberInfo>
          </Item>
          <Item>
            <MemberInfo>
              <MemberName>Vaibhav Ashokbhai Gajera </MemberName>
              <MemberRole>Embedded Software and Hardware Engineer </MemberRole>
              <LinkedInLink
                href="https://www.linkedin.com/in/vaibhavgajera23/"
                target="_blank"
                rel="noopener noreferrer"
              >
                <LinkedInIcon
                  viewBox="0 0 24 24"
                  xmlns="http://www.w3.org/2000/svg"
                >
                  <path d="M20.447 20.452h-3.554v-5.569c0-1.328-.027-3.037-1.852-3.037-1.853 0-2.136 1.445-2.136 2.939v5.667H9.351V9h3.414v1.561h.046c.477-.9 1.637-1.85 3.37-1.85 3.601 0 4.267 2.37 4.267 5.455v6.286zM5.337 7.433c-1.144 0-2.063-.926-2.063-2.065 0-1.138.92-2.063 2.063-2.063 1.14 0 2.064.925 2.064 2.063 0 1.139-.925 2.065-2.064 2.065zm1.782 13.019H3.555V9h3.564v11.452zM22.225 0H1.771C.792 0 0 .774 0 1.729v20.542C0 23.227.792 24 1.771 24h20.451C23.2 24 24 23.227 24 22.271V1.729C24 .774 23.2 0 22.222 0h.003z" />
                </LinkedInIcon>
                LinkedIn
              </LinkedInLink>
            </MemberInfo>
          </Item>
        </MembersList>
      </Section>
    </Page>
  );
}
